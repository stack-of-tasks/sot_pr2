#include "sot_pr2/pr2_controller_plugin.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

boost::condition cond2;
boost::mutex mut;

struct timeval t0, t1, t2, t3;

void
controller(sot_pr2::Pr2ControllerPlugin *pr2) {
    // Setup
    gettimeofday(&t0,0);
    pr2->starting();

    while(true) {
        gettimeofday(&t1, 0);
        long elapsed = (t1.tv_sec-t0.tv_sec)*1000000 + t1.tv_usec-t0.tv_usec;
        if (elapsed >= 1000) {
            t0 = t1;
            gettimeofday(&t2,0);
            pr2->update();
            gettimeofday(&t3,0);
            long elapsed2 = (t3.tv_sec-t2.tv_sec)*1000000 + t3.tv_usec-t2.tv_usec;
            std::cout << "[FakeController] : " << elapsed2 << " µs" << std::endl;
        }
        //boost::this_thread::sleep(boost::posix_time::microseconds(1));
    }

    pr2->stopping();
}

int main(void) {
    sot_pr2::Pr2ControllerPlugin pr2;
    ros::NodeHandle n("sot_pr2_fake_controller");
    pr2.init(0,n);
    boost::thread control_loop(controller,&pr2);
    control_loop.join();
}
