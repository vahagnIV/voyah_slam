# Bag of Words without OpenCV — Design Spec

Date: 2026-10-07
Status: approved in conversation, pending spec review
Roadmap: step 2 in `docs/ROADMAP.md`

## Context and Scope

Place recognition (relocalization, loop detection) and fast keyframe-to-
keyframe matching need a binary bag-of-words vocabulary. ORB-SLAM3 uses
DBoW2, which depends on OpenCV for descriptor storage (`cv::Mat`) and small
helpers. Our ORB descriptors already use ORB-SLAM's 512-point BRIEF pattern
and bit order, so ORB-SLAM3's pretrained `ORBvoc.txt` applies.

**Licensing is deferred (user decision).** The software runs on the user's
PC, and `ORBvoc.txt` is used as is; it is never committed. Vocabulary
training is out of scope.

**In scope:**

- an OpenCV-free DBoW2 port: vocabulary, transform and L1 scoring;
- the text loader, our binary format and a converter tool;
- vocabulary ownership and wiring into the Mapper (BoW per Frame);
- `SearchByBoW` for triangulation;
- the ORB rotated-pattern rounding fix.

**Out of scope:**

- training;
- the Frame database and relocalization (step 3);
- loop detection (step 4);
- BoW in the Tracker;
- licensing.

**Success:**

- our transform reproduces reference DBoW2 output on real descriptors;
- the Mapper's triangulation time per Frame drops at least 3×;
- the corridor1 smoke results are no worse than the latest DESIGN table (one
  mono map; stereo with no loss).

## 1. Module

The code lives in `voyah_algs`, under `bow/`, namespace `voyahslam::bow`.
Descriptor operations come from the extractor type: `FE::TDescriptor`,
`FE::TDescriptors` (columns) and `FE::Distance`. There is no OpenCV.

```cpp
using WordId = uint32_t;  using NodeId = uint32_t;
struct BowVector     { std::vector<std::pair<WordId, double>> entries; };   // sorted by word, L1-normalised
struct FeatureVector { std::vector<NodeId> nodes;                           // sorted
                       std::vector<uint32_t> offsets, features; };          // CSR: features of nodes[i]

template <typename FE> class Vocabulary {
public:
    static std::optional<Vocabulary> LoadText(const std::string& path);
    static std::optional<Vocabulary> LoadBinary(const std::string& path);
    [[nodiscard]] bool SaveBinary(const std::string& path) const;
    static Vocabulary FromTree(int k, int L, std::vector<NodeId> parents,
                               typename FE::TDescriptors descriptors, std::vector<double> weights);
    void Transform(const typename FE::TDescriptors& descriptors, int levels_up,
                   BowVector& out_bow, FeatureVector& out_features) const;
    [[nodiscard]] static double Score(const BowVector& a, const BowVector& b);
    [[nodiscard]] size_t num_words() const;  [[nodiscard]] int k() const;  [[nodiscard]] int L() const;
};
```

**Storage.**
- Nodes are kept in id order, as arrays: descriptors (one column per node),
  parent, children in CSR form, weight (idf), and word id (−1 for inner
  nodes).
- Children and word ids are rebuilt from the parents at load.
- Node 0 is the root.

**Transform** (DBoW2 TF-IDF, L1):
- Each descriptor descends from the root, at every node taking the child with
  the smallest Hamming distance. On a tie, the first child in stored order
  wins.
- The leaf it reaches is its word.
- Each occurrence of a word adds that word's idf weight. The BowVector is
  then L1-normalised.
- The FeatureVector files the feature index under the word's ancestor
  `levels_up` levels above the leaf, or under the root if the tree is
  shallower.

**Score** (DBoW2 `L1Scoring`): over the words both vectors share,
`s = Σ (|a| + |b| − |a − b|)`, and the result is `s / 2`, in [0, 1] for
normalised input.

**Load rules.**
- Only L1 scoring and TF-IDF weighting are accepted; the other DBoW2 types
  are rejected.
- Loaders never throw. On failure they return `std::nullopt` and log the
  reason.

## 2. Files

**Text format** (DBoW2 / ORB-SLAM):
- The header line is `k L scoring weighting`.
- Each following line is a node, in id order from 1:
  `parent is_leaf d0 … d31 weight`, with the descriptor as decimal bytes.

**Binary format** (ours, little-endian):

```
magic "VOYBOW\0\1" | u32 k | u32 L | u32 descriptor_bytes | u64 node_count
| u32 parent[node_count] | u8 descriptor[node_count][descriptor_bytes] | f64 weight[node_count]
| u64 checksum (FNV-1a 64 of all preceding bytes)
```

- A wrong magic, a wrong descriptor size, a truncated file or a bad checksum
  makes the load fail.
- `tools/convert_vocabulary <in.txt> <out.bin>` loads the text file, saves
  the binary file, and reloads it to verify that the two trees are identical.
- The vocabulary file is never committed.

## 3. Wiring

- **Apps:** both apps take `--vocabulary <file.bin>`, load it once, and pass
  a `std::shared_ptr<const bow::Vocabulary<FE>>` to `Slam`, which passes it
  to the Mapper.
  - The vocabulary is immutable, so threads may share it without locks.
  - The pointer may be null.
- **Mapper, with a vocabulary:** every Frame, the initial ones included,
  stores the `BowVector` and `FeatureVector` (`levels_up` = 4) of its left
  descriptors. These are not published.
- **Without a vocabulary,** behaviour is exactly as today.

## 4. `SearchByBoW` for triangulation

`mapping::SearchForTriangulation` gains two optional
`const bow::FeatureVector*` parameters.

- **When both are given,** candidate pairs come only from features filed
  under the same node in both Frames. The two sorted node lists are walked
  side by side.
- **All existing checks are unchanged:** no point yet, distance ≤ 50, ratio
  0.6, epipolar band on rays, rotation histogram, one-to-one.
- **Without them,** brute force runs as today.
- Fusion and the Tracker's projection matching are not changed.

## 5. ORB rotated-pattern rounding

The rotated BRIEF offsets use `std::lround` instead of truncation, matching
ORB-SLAM3's `cvRound`. Tests that pin exact descriptor bits are updated.

## 6. Testing

**Unit tests (`voyah_algs`)**, using a hand-built vocabulary (k = 3, L = 2):

- `Transform`:
  - closest child at every level, ties to the first child;
  - idf weights summed and L1-normalised;
  - `levels_up` ancestor correct, and the root used for a shallow tree.
- `Score`: identical vectors give 1; disjoint vectors give 0; one
  hand-computed value.
- Binary format:
  - a round trip gives an identical tree;
  - a bad magic, checksum or descriptor size is rejected;
  - a truncated file is rejected.
- Text loader: a tiny DBoW2-format fixture parses; other scoring or weighting
  types are rejected.
- `SearchByBoW`:
  - its result is a subset of brute force on the same views;
  - with every feature under one node it equals brute force exactly.

**Equivalence with DBoW2.**
- The original DBoW2 (BSD; ORB-SLAM3 `Thirdparty/DBoW2` with OpenCV) is built
  in a scratch directory outside the repository.
- It is run on our extractor's descriptors of 3 corridor1 frames. The output
  is committed as a fixture (about 100 KB): the descriptors as raw bytes, and
  the expected BowVector and FeatureVector.
- `VocabularyEquivalenceTest` loads `ORBvoc.txt` from `$VOYAHSLAM_ORBVOC` and
  must match exactly: the same words, the same feature filing, and weights
  within 1e-12. When the variable is unset it skips.
- Its label is `vocabulary`, and it is not part of `check`.

**`voyah_slam` tests:**
- with a tiny vocabulary, every Frame has BoW data and triangulation takes
  the BoW path;
- with a null vocabulary, everything is unchanged.

**Smoke runs** (corridor1, Release, converted `ORBvoc`):

| Run | Must | Report |
|---|---|---|
| Mono 400 | One map; triangulation time per Frame at least 3× lower than without a vocabulary | Mapper ms per pass, BA ms, Frames, points, ATE vs the latest DESIGN table |
| Stereo 1000 | No loss, one map; the same 3× | same |

## 7. Docs

- **`DESIGN.md`:** the bow module, the binary format, vocabulary ownership,
  and decisions-log entries.
- **`TODO.md`:** vocabulary training, the `ORBvoc.txt` licence, and the Frame
  database (step 3).
- **`README.md`:** how to obtain `ORBvoc.txt` (ORB-SLAM3 repository) and
  convert it.
- **`ROADMAP.md`:** step 2 marked done.
