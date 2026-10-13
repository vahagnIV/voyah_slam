#ifndef VOYAHSLAM_ERRORS_H
#define VOYAHSLAM_ERRORS_H

#include <stdexcept>

namespace voyahslam {

// The library's usage error: an invalid call sequence or a call from the wrong
// state (connecting to a producer that runs, resetting a Subscription while its
// producer runs, ...). Thrown in every build, never only asserted, because the
// caller (the JNI binding, a C++ app) can recover or at least report it. The JNI
// binding maps it to IllegalStateException (DESIGN decisions log, D-09).
//
// Destructors never throw: a misuse that can only be detected in a destructor
// (a Subscription destroyed while its producer runs) logs and terminates instead.
class UsageError : public std::logic_error {
public:
    using std::logic_error::logic_error;
};

} // namespace voyahslam

#endif // VOYAHSLAM_ERRORS_H
