// Prints which OpenCV tracker classes the linked OpenCV provides. Build: see jetson_build_opencv.sh (stage "probe").
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#if __has_include(<opencv2/tracking.hpp>)
#include <opencv2/tracking.hpp>
#define HAVE_CONTRIB_TRACKING 1
#endif
#include <iostream>
int main() {
    std::cout << "OpenCV " << CV_VERSION << "\n";
    cv::Mat f(240, 320, CV_8UC3, cv::Scalar(90, 120, 150)); cv::Rect r(100, 80, 60, 60);
    auto probe = [&](const char* name, auto make) {
        try { auto t = make(); t->init(f, r); cv::Rect b = r; bool ok = t->update(f, b); std::cout << "  " << name << ": available (init/update ran, ok=" << ok << ")\n"; }
        catch (const cv::Exception& e) { std::cout << "  " << name << ": present but failed: " << e.msg << "\n"; }
    };
    probe("MIL", [] { return cv::TrackerMIL::create(); });
#ifdef HAVE_CONTRIB_TRACKING
    probe("KCF", [] { return cv::TrackerKCF::create(); });
    probe("CSRT", [] { return cv::TrackerCSRT::create(); });
#else
    std::cout << "  KCF/CSRT: contrib tracking module not built\n";
#endif
    std::cout << "  Nano/Vit: classes exist in this build, but need ONNX model files to run (see README)\n";
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    std::cout << "  TrackerNano class: compiled in (version >= 4.7)\n";
#endif
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 9)
    std::cout << "  TrackerVit class: compiled in (version >= 4.9)\n";
#endif
    return 0;
}
