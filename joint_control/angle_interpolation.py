'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import rightBellyToStand
from scipy import interpolate


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(
            simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        if(keyframes == ([], [], [])):
            return {}
        else:
            (names, times, keys) = keyframes

        if (self.start == -1):
            self.start = perception.time

        timeInterval = perception.time - self.start
        endIndex = 0

        for (nameIterator, name) in enumerate(names):
            timeFrame = times[nameIterator]
            upperThreshold = 0
            lowerThreshold = 0
            skippedJoints = 0

            if timeFrame[-1] < timeInterval:
                skippedJoints += 1
                if(skippedJoints == len(names)):
                    self.start = -1
                    self.keyframes = ([], [], [])
                continue
            endIndex,upperThreshold, lowerThreshold = self.findTimeSpan(times, nameIterator, upperThreshold, timeFrame, lowerThreshold, timeInterval, endIndex)

            if (upperThreshold - lowerThreshold) == 0:
                t = 1
            else:
                t = (timeInterval - lowerThreshold) /  (upperThreshold - lowerThreshold)
                if(t > 1):
                    t = 1
                elif t < 0:
                    t = 0

            if (endIndex != 0):
                p0, p1, p2, p3 = self.setPoints(keys, nameIterator, endIndex)
            elif (endIndex == 0):
                p0, p1, p2, p3 = self.resetPoints(keys, nameIterator)

            angle = ((1-t)**3)*p0 + 3*t*((1-t)**2) * \
                p1 + 3*(t**2)*(1-t)*p2 + (t**3)*p3

            if(name == "LHipYawPitch"):
                target_joints["RHipYawPitch"] = angle

            target_joints[name] = angle

        return target_joints

    def resetPoints(self, keys, nameIterator):
        p0 = 0
        p1 = 0
        p3 = keys[nameIterator][0][0]
        p2 = p3 + keys[nameIterator][0][1][2]
        return p0, p1, p2, p3

    def setPoints(self, keys, nameIterator, endIndex):
        p0 = keys[nameIterator][endIndex-1][0]
        p3 = keys[nameIterator][endIndex][0]
        p1 = p0 + keys[nameIterator][endIndex-1][1][2]
        p2 = p3 + keys[nameIterator][endIndex][1][2]
        return p0, p1, p2, p3

    def findTimeSpan(self, times, nameIterator, upperThreshold, timeFrame, lowerThreshold, timeInterval, endIndex):
        for timeIterator in range(len(times[nameIterator])):
            upperThreshold = timeFrame[timeIterator]
            if (lowerThreshold <= timeInterval and upperThreshold > timeInterval):
                endIndex = timeIterator
                break
            lowerThreshold = upperThreshold
        return endIndex, upperThreshold, lowerThreshold


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
