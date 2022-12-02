import numpy as np
import math

class KalmanFilter:

    def __init__(self, x=0, y=0, theta=0):
        self.s = np.array([x, y, theta])
        
        self.landmark_order = np.array([])
        self.number_of_landmarks = {}
        self.landmark_count = 0

        # Initially we assume we know with certainity that the position of the landmark is correct.
        # TODO: Would have to change sigma initialization if the assumption fails
        self.sigma = np.zeros((3, 3))#keep it low values #3+3N square matrix

        self.Kt = np.identity(3)# size: (3N+3)x3
        self.St = np.identity(3) # size: mp 3x3

        self.camera_noise = 0.3
        self.motion_noise = 0.1
        # TODO: Would need to change initialization of Rt and Qt to low values and not perfect 0 or 1. Maybe Gaussian?
        self.Rt = self.camera_noise*np.identity(3)#  size always: 3x3 # should be empty initially
        self.Qt = self.motion_noise*np.identity(3)# size: (3+3N, 3+3N)

        #self.zt = np.array()
        self.F = np.identity(len(self.s))
        self.G = np.identity(3)# len(s) X 3

        self.H = self.H_theta(self.s[2], landmark=None)

    def H_theta(self, theta, landmark=None):
        # Update the values only corresponding to the landmark passed. Rest keep as same.
        # Here landmark indicates the april tag number. We convert this automatically to the actual appropriate matrix index
        N = self.landmark_count
        F_xj = np.zeros((6, 3*N+3))
        F_xj[0:3, 0:3] = np.identity(3)

        if landmark is not None:
            i = np.where(self.landmark_order==landmark)[0][0]
            F_xj[3:6, (3*i+3):(3*i+3+3)] = np.identity(3)
        
        H_low = np.array([
            [-math.cos(theta), -math.sin(theta), 0, math.cos(theta), math.sin(theta), 0], 
            [math.sin(theta), -math.cos(theta), 0, -math.sin(theta), math.cos(theta), 0],
            [0, 0, -1, 0, 0, 1]])
        
        return np.matmul(H_low, F_xj)
    
    def addLandmark(self, z, tag):
        print("Added landmark!!!!!!!")
        self.s = np.append(self.s, z)
        
        self.landmark_count += 1
        self.landmark_order = np.append(self.landmark_order, tag)
        
        # Sigma - error computation
        # TODO: Do sigma updation appropriately based on validity of assumption mentioned in prev function
        # Also, the matrix in temp2 would not be identity or a zero perfectly. Maybe low values on diagonal?
        sigma_size = self.sigma.shape[0]
        temp1 = np.hstack((self.sigma, np.zeros((sigma_size, 3))))
        temp2 = np.hstack((np.zeros((3, sigma_size)), (self.camera_noise+self.motion_noise)*np.identity(3)))
        self.sigma = np.vstack((temp1, temp2))
        
        #self.Kt - no need to change this. computed at run-time when needed
        #self.St - no need to change this. computed at run-time when needed

        # TODO: Update Rt and Qt appropriately
        self.Rt = self.camera_noise*np.identity(3*self.landmark_count)# No need as it is always 3x3: sensor noise
        self.Qt = self.motion_noise*np.identity(3+3*self.landmark_count)# Not sure about size (3+3N, 3+3N)

        self.F = np.identity(len(self.s)) 
        self.G = np.vstack((self.G, np.zeros((3, 3))))

        ## This line below is not needed. Can comment this anytime
        self.H = self.H_theta(self.s[2], landmark=tag)

    def update(self, zt, tag=None):
        # zt: Pose of april tag w.r.t. robot (but earlier it is w.r.t camera)
        # tag: april tag's number. Pass None is no tag is found. Accordinly the H matrix is called
        #if tag not in self.landmark_order:
        #    self.addLandmark(zt, tag)
        
        theta = self.s[2]
        self.H = self.H_theta(theta, landmark=tag)

        self.St = np.matmul(self.H, np.matmul(self.sigma, self.H.T)) + self.Rt
        self.Kt = np.matmul(self.sigma, np.matmul(self.H.T, np.linalg.inv(self.St)))
        self.s = self.s + np.matmul(self.Kt, (zt - np.matmul(self.H, self.s)))# s_t|t
        self.sigma = np.matmul(np.identity(len(self.sigma)) - np.matmul(self.Kt, self.H), self.sigma)

    def return_selected_rt(self, tags):
        landmark_indices = np.argwhere(np.isin(self.landmark_order, tags)).reshape(-1)
        indices = []
        for ind in landmark_indices:
            indices = indices + [3*ind, 3*ind+1, 3*ind+2]
        return self.Rt[indices, :][:, indices]
    
    def angle_correction(self, v):
        # assumption: v is of length 3n:
        n = len(v)//3
        for i in range(n):
            while True:
                if v[(3*i+2)]>np.pi:
                    v[(3*i+2)] -= 2*np.pi
                elif v[(3*i+2)]<-np.pi:
                    v[(3*i+2)] += 2*np.pi
                else:
                    break
        return v
    
    def updates(self, zts, tags=None):
        # zt: Pose of april tag w.r.t. robot (but earlier it is w.r.t camera)
        # tag: april tag's number. Pass None is no tag is found. Accordinly the H matrix is called
        #if tag not in self.landmark_order:
        #    self.addLandmark(zt, tag)
        
        theta = self.s[2]
        self.H = np.vstack((self.H_theta(theta, landmark=landmark) for landmark in tags))
        zts = zts.reshape(-1)

        self.St = np.matmul(self.H, np.matmul(self.sigma, self.H.T)) + self.return_selected_rt(tags)
        self.Kt = np.matmul(self.sigma, np.matmul(self.H.T, np.linalg.inv(self.St)))

        self.s = self.s + np.matmul(self.Kt, self.angle_correction(zts - np.matmul(self.H, self.s)))# s_t|t
        self.s = self.angle_correction(self.s)

        self.sigma = np.matmul(np.identity(len(self.sigma)) - np.matmul(self.Kt, self.H), self.sigma)

    def predict(self, ut):
        # Assumption: ut is the velocity in a numpy array. size: (3)
        self.s = np.matmul(self.F, self.s) + np.matmul(self.G, ut)
        self.s = self.angle_correction(self.s)

        self.sigma = np.matmul(self.F, np.matmul(self.sigma, self.F.T)) + self.Qt

    def getCurrentPos(self):
        return self.s[0:3]

if __name__ == "__main__":
    # code to test if the Kalman filter works
    kf = KalmanFilter()
    kf.predict(np.array([1, 1, 1]))
    kf.addLandmark(np.array([1, 1, 1]), tag=1)
    kf.addLandmark(np.array([1, 1, 1]), tag=2)
    kf.addLandmark(np.array([1, 1, 1]), tag=3)
    kf.updates(np.array([np.array([1, 1, 1]), np.array([1, 1, 1])]), tags=[1, 2])
    # kf.predict(np.array([1, 1, 1]))
    # kf.update(np.array([1, 1, 1]), tag=2)
    # kf.predict(np.array([1, 1, 1]))
    # kf.update(np.array([1, 1, 1]), tag=1)

    