# DBoW2 reference fixture

`libs/algs/tests/bow/data/dbow2_reference.bin` is the expected output of the
original DBoW2 for `VocabularyEquivalenceTest`
(`libs/algs/tests/bow/vocabulary_equivalence_test.cpp`, binary
`voyah_vocabulary_tests`, ctest label `vocabulary`). The programs that made it
were built in a scratch directory and are not committed. This file describes
how to rebuild them.

Run the test with the real vocabulary:

```
VOYAHSLAM_ORBVOC=/path/to/ORBvoc.txt ctest -L vocabulary
```

Without `VOYAHSLAM_ORBVOC` the test is skipped.

## Inputs

- **ORB-SLAM3:** https://github.com/UZ-SLAMLab/ORB_SLAM3, shallow clone at
  commit `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`. It supplies
  `Thirdparty/DBoW2` (BSD) and `Vocabulary/ORBvoc.txt.tar.gz`.
  - The extracted `ORBvoc.txt` is 145 250 924 bytes, with sha256
    `f8dd027f7a6cb88129821341194d7f2c75b77b3394257ddd0d2229863d1a3570`.
- **Images:** EuRoC-format corridor1 (`dataset-corridor1_1024_16`),
  `mav0/cam0/data`. We use the files at indices 0, 200 and 400 in sorted
  filename order:
  - `1520531829251142058.png`;
  - `1520531839251462005.png`;
  - `1520531849251631286.png`.

## Step 1: descriptors (our extractor)

A small program linking `voyah_algs` (Release) does the following for each image:
1. reads it with `io::ReadImageFromStream`;
2. runs `features::OrbFeatureExtractor{}` (defaults: scale 1.2, 8 levels,
   FAST thresholds 20/7) with `DetectAndExtract(image, 1000)`;
3. writes the descriptors to `descriptors.bin` as
   `u32 num_frames | per frame: u32 n | u8 desc[n][32]` (little-endian, one
   32-byte descriptor per feature, in extractor order).

This gives 1005, 1001 and 1002 features.

## Step 2: DBoW2 reference program

DBoW2 builds against the system OpenCV 5.1 (`/usr/local`). We did not use its
CMakeLists, which asks for OpenCV 4 or 3. Instead we compiled the six sources
directly:

```
g++ -std=c++17 -O2 -I/usr/local/include/opencv5 -IThirdparty/DBoW2 -c \
    DBoW2/{BowVector,FORB,FeatureVector,ScoringObject}.cpp DUtils/{Random,Timestamp}.cpp
```

**Patch** (one only; no algorithm change): `DBoW2/BowVector.h` and
`DBoW2/FeatureVector.h` include Boost.Serialization. ORB-SLAM3 added this to
save its Atlas, and Boost is not installed. We deleted the two
`#include <boost/serialization/...>` lines and the `friend class
boost::serialization::access; template<class Archive> void serialize(...)`
member from both classes. DBoW2 needed no other change for OpenCV 5.

The reference program does the following:
1. loads `ORBvoc.txt` with `ORBVocabulary::loadFromTextFile`, where
   `ORBVocabulary` is
   `TemplatedVocabulary<FORB::TDescriptor, FORB>`, as in ORB-SLAM3;
2. turns each frame's descriptors into a `std::vector<cv::Mat>` of 1×32
   `CV_8U` rows, as `Converter::toDescriptorVector` does;
3. calls `transform(desc, bow, feat, 4)`;
4. writes the fixture (little-endian):

```
u32 num_frames | per frame: u32 n | u8 desc[n][32] | u32 bow_count | (u32 word, f64 weight)[bow_count]
               | u32 node_count | per node: u32 node | u32 m | u32 features[m]
```

`bow` and `feat` are written in their `std::map` order, which is ascending
word and ascending node. Within a node, features are in insertion order,
which is ascending.

The output is 144 680 bytes, with sha256
`472a7bba2a74ebfc06d180356267b9e0f30ec8516635ae68c2e0cce66e5733ff`.

## Findings from the reference run

- **Phantom node.** `ORBvoc.txt` ends with a single `\n` after its last node
  line. DBoW2's `while(!f.eof()) getline` loop therefore reads one more empty
  line and appends a phantom node:
  - DBoW2 has 1 082 074 nodes and 971 815 words. Ours has 1 082 073 nodes
    (with the root) and 971 814 words.
  - The phantom's parse is undefined behaviour (uninitialised `pid` and
    `nIsLeaf`). In this build it became the last child of node 1 081 982,
    which is the last line's parent. It is a leaf with word id 971 814,
    weight 0 and an all-zero descriptor.
  - No fixture feature descends through node 1 081 982, so the fixture is
    unaffected.
  - In general, a feature reaching that node whose closest child is the
    phantom would be dropped by DBoW2 (weight 0), but filed under a real
    sibling by us.
- **Shallow leaves.** ORBvoc's leaf depths are: 1 at depth 4, 819 at depth 5
  and 970 995 at depth 6. None is shallower than L − 4 = 2, so the
  shallow-leaf filing ruling never applies with ORBvoc and `levels_up` 4.
- **Distance 256.** No child distance evaluated during the descents was 256,
  so `OrbFeatureExtractor::Distance` returning `uint8_t` changes no word.
- 90 609 leaves have weight 0. The fixture frames have 2, 7 and 5 features
  that land on one. Both implementations skip these features.
