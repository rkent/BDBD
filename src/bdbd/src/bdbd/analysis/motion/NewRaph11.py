# newton-raphson iteration of motion equations

import numpy as np
import time
import rospy
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R, transform2d
from bdbd_common.NewRaph import NewRaph
from bdbd_common.msg import LrOptimizeGoal, LrOptimizeAction, LrResults
import actionlib
from actionlib_msgs.msg import GoalStatus
from bdbd_common.pathPlan2 import PathPlan
import matplotlib.pyplot as plt
try:
    from Queue import Queue
except:
    from queue import Queue

def estr(a):
    return fstr(a, fmat='8.4g', n_per_line=10)

class LrOptimizeClient:
    def __init__(self,
        Wmax=1.0e-2,
        Wjerk=1.0e-2,
        Wback=1.0,
        mmax=1.0,
        queue=Queue()
    ):
        rospy.loginfo('init LrOptimizeClient')
        self.Wmax = Wmax
        self.Wjerk = Wjerk
        self.Wback = Wback
        self.mmax = mmax
        self.queue = queue
        self.goal = None

    def done_cb(self, state, r):
        rospy.loginfo(gstr({'done state': GoalStatus.to_string(state), 'elapsed time': time.time() - self.start, 'loss': r.loss}))
        if state == GoalStatus.SUCCEEDED:
            self.queue.put(r)

    def feedback_cb(self, feedback):
        if self.queue.empty():
            self.queue.put(feedback)
        pass

    def __call__(self, dt, lefts, rights
        ,gaussItersMax=0
        ,nrItersMax=1
        ,start_pose=(0.0, 0.0, 0.0)
        ,start_twist=(0.0, 0.0, 0.0)
        ,target_pose=(0.0, 0.0, 0.0)
        ,target_twist=(0.0, 0.0, 0.0)
        ,target_lr=(0.0, 0.0)
        ,rate=20

    ):
        self.start = time.time()
        self.client = actionlib.SimpleActionClient('/bdbd/dynamicPath', LrOptimizeAction)
        self.client.wait_for_server()
        self.feedback_sub = rospy.Subscriber('/bdbd/dynamicPath/feedsub', LrResults, self.feedback_cb)

        goal = LrOptimizeGoal()
        goal.dt = dt
        goal.start_lefts = lefts
        goal.start_rights = rights
        goal.start_pose = start_pose
        goal.start_twist = start_twist
        goal.target_pose = target_pose
        goal.target_twist = target_twist
        goal.target_lr = target_lr
        goal.Wmax = self.Wmax
        goal.Wjerk = self.Wjerk
        goal.Wback = self.Wback
        goal.mmax = self.mmax
        goal.gaussItersMax = gaussItersMax
        goal.nrItersMax = nrItersMax
        goal.rate = rate
        self.goal = goal
        self.client.send_goal(goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)
        rospy.loginfo('waiting for result')

class PathPlot():
    def __init__(self, history_rate=5):
        self.fig = plt.figure(figsize=(12,4))
        self.base_pxj = None
        self.base_pyj = None
        self.base_frame = None
        self.history_pxj_base = []
        self.history_pyj_base = []
        self.count = 0
        self.plan_history = []
        self.history_rate = history_rate
    def __call__(self, dt, source):
        self.count += 1
        if self.base_frame is None:
            # save the first predicted trajectory and frame
            self.base_pxj = result.pxj
            self.base_pyj = result.pyj
            self.base_frame = result.now_pose_map

        fig = self.fig
        s = source
        tees = [0.0]
        tt = 0.0
        for i in range(1, len(s.lefts)):
            tt += dt
            tees.append(tt)

        fig.clf()
        plt1 = fig.add_subplot(131)
        #plt1.axis([0.0, tfPath.lrs[-1]['t'], -1.5, 1.5])
        plt2 = fig.add_subplot(132)
        plt3 = fig.add_subplot(133)

        #if axis3 is not None:
        #    plt3.axis(axis3)

        plt2.axis('equal')

        plt1.plot(tees, s.lefts)
        plt1.plot(tees, s.rights)
        plt1.set_title('left,right')
        # plt1.plot(s.tees, s.omegaj)

        # Transform to base_frame coordinates
        pxj_base = []
        pyj_base = []
        for i in range(len(s.pxj)):
            p_robot = (s.pxj[i], s.pyj[i], s.thetaj[i])
            p_base = transform2d(p_robot, s.now_pose_map, self.base_frame)
            pxj_base.append(p_base[0])
            pyj_base.append(p_base[1])

        now_pose_base = transform2d(s.now_pose_map, (0., 0., 0.), self.base_frame)
        self.history_pxj_base.append(now_pose_base[0])
        self.history_pyj_base.append(now_pose_base[1])

        if self.count % self.history_rate == 0:
            self.plan_history.append((pxj_base, pyj_base))
        else:
            plt2.plot(pxj_base, pyj_base)
        for plan in self.plan_history:
            plt2.plot(plan[0], plan[1])
        #plt2.plot(self.base_pxj, self.base_pyj)
        plt2.plot(pxj_base, pyj_base)
        plt2.plot(self.history_pxj_base, self.history_pyj_base, 'r+')
        plt2.set_title('px vs py')

        plt3.plot(tees, s.pxj)
        plt3.plot(tees, s.pyj)
        plt3.set_title('px,py vs t')
        plt.pause(.0001)

if __name__ == '__main__':

    dt = 0.02
    lr_model = default_lr_model()
    #lr_model = ((1.0, 1.0, 10.0), (-1.0, 1.0, 10.0), (-1.0, 10.0, 10.0))
    start_pose = [0.0, 0.0, 0.0]
    start_twist = [0.0, 0.0, 0.0]
    target_pose = [0.3, 0.1, D_TO_R * 90]
    target_twist = [0.0, 0.0, 0.0]
    approach_rho = 0.05
    min_rho = 0.02
    cruise_v = 0.25
    lr_start = (0.0, 0.0)
    gauss_iters = 20
    nr_iters = 20
    Wmax = 1.e-6
    #Wmax = 0.0
    Wjerk = 1.e-2
    Wback = 1.0
    #Wback = 0.0
    mmax = 1.0
    NRstart = 1.0
    NRfact = 2
    maxSlew = 2.00
    testNR = False
    maxN = 50
    rate = 40

    rospy.init_node('NewRaph', log_level=rospy.DEBUG)

    nr = NewRaph()
    for count in range(3):
        (lefts, rights) = nr.initial_plan(dt
            , lr_model=lr_model
            , start_pose=start_pose
            , start_twist=start_twist
            , target_pose=target_pose
            , target_twist=target_twist
            , Wmax=Wmax
            , Wjerk=Wjerk
            , Wback=Wback
            , mmax=1.0
            , approach_rho=approach_rho
            , min_rho=min_rho
        )
        n = len(lefts)
        if n > maxN:
            dt *= n / maxN
            print('n=', n, ' too large, adjusting dt to ', dt)
        else:
            break

    pathPlot = PathPlot(history_rate=3)

    # send to C++ node for processing
    lroClient = LrOptimizeClient(Wmax=Wmax, Wjerk=Wjerk, Wback=Wback, mmax=mmax)

    lroClient(dt, lefts, rights
        , gaussItersMax=gauss_iters
        , nrItersMax=nr_iters
        , start_pose=start_pose
        , start_twist=start_twist
        , target_pose=target_pose
        , rate=rate
        )
    print('\n***** started action *****')

    gauss_count = 0
    nr_count = 0
    eps = 1.0
    while not rospy.is_shutdown():
        rospy.sleep(0.001)
        plt.pause(.001)
        if not lroClient.queue.empty():
            result = lroClient.queue.get();
            '''
            tt = 0.0
            for i in range(len(result.lefts)):
                print(fstr({
                    't': tt,
                    'lr': (result.lefts[i], result.rights[i]),
                    'pose': (result.pxs[i], result.pys[i], result.thetas[i]),
                    'twist': (result.vxs[i], result.vys[i], result.omegas[i])
                }))
                tt += dt
            nr.poses(result.lefts, result.rights)
            '''
            # print(gstr({'lefts': result.lefts, 'rights': result.rights}))
            #print(lroClient.client.get_state(), lroClient.client.get_goal_status_text())
            try:
                pathPlot(result.dt, result)
                print(fstr({'n': len(result.lefts), 'x': result.now_pose_start[0], 'y': result.now_pose_start[1], 'theta deg': result.now_pose_start[2]/D_TO_R}))
            except:
                print('Error in pathPlot')
