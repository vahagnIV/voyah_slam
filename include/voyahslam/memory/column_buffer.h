#ifndef VOYAHSLAM_MEMORY_COLUMN_BUFFER_H
#define VOYAHSLAM_MEMORY_COLUMN_BUFFER_H

#include <Eigen/Dense>
#include <algorithm>
#include <cassert>

namespace voyahslam::memory {

// A Rows × N Eigen matrix whose allocation only ever grows. Resizing to N
// columns within the current capacity never touches the heap, so a buffer that
// lives in a pooled object stops allocating once it has seen its largest batch.
template <typename TScalar, int Rows>
class ColumnBuffer {
public:
    using TStorage = Eigen::Matrix<TScalar, Rows, Eigen::Dynamic>;

    // Sets the number of visible columns. Reallocates (geometrically) only when
    // `cols` exceeds capacity; the first min(old, new) columns are preserved.
    void Resize(Eigen::Index cols) {
        assert(cols >= 0);
        if (cols > storage_.cols()) {
            storage_.conservativeResize(Eigen::NoChange, std::max(cols, 2 * storage_.cols()));
        }
        cols_ = cols;
    }
    void Clear() { cols_ = 0; }

    [[nodiscard]] auto matrix() { return storage_.leftCols(cols_); }
    [[nodiscard]] auto matrix() const { return storage_.leftCols(cols_); }
    [[nodiscard]] auto col(Eigen::Index idx) {
        assert(idx < cols_ && "ColumnBuffer column out of range");
        return storage_.col(idx);
    }
    [[nodiscard]] auto col(Eigen::Index idx) const {
        assert(idx < cols_ && "ColumnBuffer column out of range");
        return storage_.col(idx);
    }
    [[nodiscard]] Eigen::Index cols() const { return cols_; }
    [[nodiscard]] Eigen::Index capacity() const { return storage_.cols(); }

private:
    TStorage storage_;
    Eigen::Index cols_ = 0;
};

} // namespace voyahslam::memory

#endif // VOYAHSLAM_MEMORY_COLUMN_BUFFER_H
