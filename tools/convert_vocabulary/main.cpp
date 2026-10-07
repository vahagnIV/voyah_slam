// Converts a DBoW2 / ORB-SLAM text vocabulary to the checksummed binary format, then reloads
// the result and verifies that the two trees are identical.
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/features/orb_feature_extractor.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>

using Vocabulary = voyahslam::bow::Vocabulary<voyahslam::features::OrbFeatureExtractor>;

namespace {

bool SameTree(const Vocabulary& a, const Vocabulary& b) {
    return a.k() == b.k() && a.L() == b.L() && a.num_words() == b.num_words() && a.parents() == b.parents() &&
           a.weights() == b.weights() && a.descriptors() == b.descriptors();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s <in.txt> <out.bin>\n", argv[0]);
        return EXIT_FAILURE;
    }
    const auto start = std::chrono::steady_clock::now();
    const auto text = Vocabulary::LoadText(argv[1]);
    if (!text) {
        std::fprintf(stderr, "failed to load text vocabulary '%s'\n", argv[1]);
        return EXIT_FAILURE;
    }
    const double load_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (!text->SaveBinary(argv[2])) {
        std::fprintf(stderr, "failed to write '%s'\n", argv[2]);
        return EXIT_FAILURE;
    }
    const auto binary = Vocabulary::LoadBinary(argv[2]);
    if (!binary) {
        std::fprintf(stderr, "failed to reload '%s'\n", argv[2]);
        return EXIT_FAILURE;
    }
    if (!SameTree(*text, *binary)) {
        std::fprintf(stderr, "reloaded binary differs from the text vocabulary\n");
        return EXIT_FAILURE;
    }
    std::printf("converted %s -> %s: k=%d L=%d, %zu nodes, %zu words (text loaded in %.2f s), verified identical\n",
                argv[1], argv[2], text->k(), text->L(), text->num_nodes(), text->num_words(), load_seconds);
    return EXIT_SUCCESS;
}
