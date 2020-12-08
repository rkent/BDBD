# newton-raphson iteration of motion equations

import numpy as np
import time
import rospy
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R
from bdbd_common.NewRaph import NewRaph
from bdbd_common.msg import LrOptimizeGoal, LrOptimizeAction, LrResult
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
        Wmax=1.0e-3,
        Wjerk=1.0e-3,
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

    def done_cb(self, state, r):
        print(gstr({'done state': state, 'elapsed time': time.time() - self.start}))
        rospy.loginfo('LrOptimize done loss {:10.5g}'.format(r.loss))
        self.queue.put(r)

    def feedback_cb(self, feedback):
        #print('got feedback')
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

    ):
        self.start = time.time()
        self.client = actionlib.SimpleActionClient('/bdbd/lrTweak', LrOptimizeAction)
        self.client.wait_for_server()
        self.feedback_sub = rospy.Subscriber('dynamicPath/feedback', LrResult, self.feedback_cb)

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
        self.client.send_goal(goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)
        rospy.loginfo('waiting for result')

class PathPlot():
    def __init__(self):
        self.fig = plt.figure(figsize=(12,4))
    def __call__(self, dt, source):
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

        plt2.plot(s.pxj, s.pyj)
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
    target_pose = [0.3, .3, D_TO_R * 0]
    target_twist = [0.0, 0.0, 0.0]
    approach_rho = 0.05
    min_rho = 0.02
    cruise_v = 0.25
    lr_start = (0.0, 0.0)
    gauss_iters = 20
    nr_iters = 20
    Wmax = 1.e-5
    #Wmax = 0.0
    Wjerk = 1.e-3
    Wback = 1.0
    #Wback = 0.0
    mmax = 1.0
    NRstart = 1.0
    NRfact = 2
    maxSlew = 2.00
    testNR = False
    DO_LOCAL = False
    maxN = 100

    rospy.init_node('NewRaph')

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

    pathPlot = PathPlot()

    # send to C++ node for processing
    lroClient = LrOptimizeClient(Wmax=Wmax, Wjerk=Wjerk, Wback=Wback, mmax=mmax)

    lroClient(dt, lefts, rights
        , gaussItersMax=gauss_iters
        , nrItersMax=nr_iters
        , start_pose=start_pose
        , start_twist=start_twist
        , target_pose=target_pose
        )
    print('\n***** started action *****')

    gauss_count = 0
    nr_count = 0
    eps = 1.0
    while not rospy.is_shutdown():
        rospy.sleep(0.01) 

        if DO_LOCAL:
            lefts = lefts.copy()
            rights = rights.copy()
            nhess = len(lefts) - 1
            base_lefts = lefts.copy()
            base_rights = rights.copy()
            (pxj, pyj, thetaj, vxj, vyj, omegaj) = nr.poses(lefts, rights)

            pathPlot(dt, nr)

            loss = nr.loss(details=True)
            print('loss {} gauss_count {} nr_count {}'.format(loss, gauss_count, nr_count))
            (dpxdl, dpxdr, dpydl, dpydr) = nr.gradients()
            (dlefts, drights) = nr.jacobian()
            #print(gstr({'dlefts': dlefts, '\ndrights': drights}))

            if gauss_count < gauss_iters:
                rospy.sleep(0.01)
                #eps = 1.0
                gauss_count += 1
                slew = 0.0
                for i in range(1, n):
                    if abs(dlefts[i]) > slew:
                        slew = abs(dlefts[i])
                    if abs(drights[i]) > slew:
                        slew = abs(drights[i])
                # line search over deltax looking for best eps
                best_eps = 0.0
                best_loss = loss
                worst_eps = maxSlew / slew
                print('eps limited to ', worst_eps)
                eps = min(eps, worst_eps)
                for lcount in range(4):
                    last_eps = eps
                    for i in range(1, n):
                        lefts[i] = base_lefts[i] - eps*dlefts[i]
                        rights[i] = base_rights[i] - eps*drights[i]
                    nr.poses(lefts, rights)
                    loss = nr.loss()
                    if loss > best_loss:
                        worst_eps = eps
                    else:
                        best_eps = eps
                        best_loss = loss
                    if eps * 2 < worst_eps:
                        eps *= 2
                    else:
                        eps = 0.5 * (best_eps + worst_eps)
                    print(estr({'(G)eps': last_eps, 'loss': loss, 'best_eps': best_eps, 'worst_eps': worst_eps, 'new_eps': eps}))
                
                eps = best_eps
                for i in range(1, n):
                    lefts[i] = base_lefts[i] - eps*dlefts[i]
                    rights[i] = base_rights[i] - eps*drights[i]

            else:
                if nr_count >= nr_iters or eps == 0.0:
                    break
                nr_count += 1
                nr.seconds()
                hess = nr.hessian()
                b = np.concatenate([-nr.dlefts[1:], -nr.drights[1:]])
                deltax = np.linalg.solve(nr.hess, b)
                slew = np.amax(np.absolute(deltax))

                # line search over deltax looking for best eps
                best_eps = 0.0
                best_loss = loss
                worst_eps = min(maxSlew / slew, 2.0)
                eps = min(eps, worst_eps)
                for lcount in range(12):
                    last_eps = eps
                    lefts[1:] = base_lefts[1:] + eps * deltax[:nhess]
                    rights[1:] = base_rights[1:] + eps * deltax[nhess:]
                    nr.poses(lefts, rights)
                    loss = nr.loss()
                    if best_eps > 0.0 and lcount >= 4:
                        break
                    if loss > best_loss:
                        worst_eps = eps
                    else:
                        best_eps = eps
                        best_loss = loss
                    if eps * 2 < worst_eps:
                        eps *= 2
                    else:
                        eps = 0.5 * (best_eps + worst_eps)
                    print(estr({'(N)eps': last_eps, 'loss': loss, 'best_eps': best_eps, 'worst_eps': worst_eps, 'new_eps': eps}))
                
                eps = best_eps
                #eps = min(best_eps, 1.0)
                print('nr_count ', nr_count, 'using eps: ', eps)
                lefts[1:] = base_lefts[1:] + eps * deltax[:nhess]
                rights[1:] = base_rights[1:] + eps * deltax[nhess:]
        else:
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
                pathPlot(result.dt, result)
                print(fstr({'n': len(result.lefts), 'px[-1]': result.pxj[-1], 'py[-1]': result.pyj[-1], 'theta[-1]': result.thetaj[-1]}))
