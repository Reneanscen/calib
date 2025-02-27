#!/usr/bin/env python
# coding: utf-8
import transforms3d as tfs
import numpy as np
import math
from scipy.spatial.transform import Rotation as R  
import cv2

def tsai_hand_eye(Hcg, Hgij, Hcij):
    assert len(Hgij) == len(Hcij), "The size of Hgij and Hcij must be the same."
    n_status = len(Hgij)

    Rgij = np.zeros((3, 3))
    Rcij = np.zeros((3, 3))
    rgij = np.zeros((3, 1))
    rcij = np.zeros((3, 1))
    theta_gij = 0
    theta_cij = 0
    rngij = np.zeros((3, 1))
    rncij = np.zeros((3, 1))
    Pgij = np.zeros((3, 1))
    Pcij = np.zeros((3, 1))
    tempA = np.zeros((3, 3))
    tempb = np.zeros((3, 1))
    A = []
    b = []
    pinA = None
    Pcg_prime = np.zeros((3, 1))
    Pcg = np.zeros((3, 1))
    PcgTrs = np.zeros((1, 3))
    Rcg = np.zeros((3, 3))
    eyeM = np.eye(3)
    Tgij = np.zeros((3, 1))
    Tcij = np.zeros((3, 1))
    tempAA = np.zeros((3, 3))
    tempbb = np.zeros((3, 1))
    AA = []
    bb = []
    pinAA = None
    Tcg = np.zeros((3, 1))

    for i in range(n_status):
        Rgij = Hgij[i][:3, :3]
        Rcij = Hcij[i][:3, :3]
        rgij,_ = cv2.Rodrigues(Rgij)
        rcij,_ = cv2.Rodrigues(Rcij)
        theta_gij = np.linalg.norm(rgij)
        theta_cij = np.linalg.norm(rcij)
        rngij = rgij / theta_gij
        rncij = rcij / theta_cij
        Pgij = 2 * np.sin(theta_gij / 2) * rngij
        Pcij = 2 * np.sin(theta_cij / 2) * rncij

        tempA = skew(Pgij.flatten() + Pcij.flatten())
        tempb = Pcij - Pgij
    
        A.append(tempA)
        b.append(tempb)
    A = np.vstack(A)
    b = np.vstack(b)
    A = np.array(A,dtype=np.float64)
    b = np.array(b,dtype=np.float64)

    pinA = np.linalg.pinv(A)

    Pcg_prime = pinA @ b
    Pcg = 2 * Pcg_prime / np.sqrt(1 + np.linalg.norm(Pcg_prime)**2)
    PcgTrs = Pcg.T
    Rcg = (1 - np.linalg.norm(Pcg)**2 / 2) * eyeM + 0.5 * (Pcg@PcgTrs + np.sqrt(4 - np.linalg.norm(Pcg)**2) * skew(Pcg.flatten()))


    # Compute translation
    for i in range(n_status):
        Rgij = Hgij[i][:3, :3]
        Rcij = Hcij[i][:3, :3]
        Tgij = Hgij[i][:3, 3].reshape(-1, 1)
        Tcij = Hcij[i][:3, 3].reshape(-1, 1)
        tempAA = Rgij - eyeM
        tempbb = Rcg @ Tcij - Tgij
        # print("tempAA",tempAA)
        AA.append(tempAA)
        bb.append(tempbb)
    AA = np.vstack(AA)
    bb = np.vstack(bb)
    AA = np.array(AA,dtype=np.float64)
    bb = np.array(bb,dtype=np.float64)
    pinAA = np.linalg.pinv(AA)
    Tcg = pinAA @ bb

    Hcg[:3, :3] = Rcg
    Hcg[:3, 3] = Tcg.flatten()
    Hcg[3, :3] = [0, 0, 0]
    Hcg[3, 3] = 1

    return Hcg


def skew(v):
    """Skew-symmetric matrix from vector v."""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]],dtype=np.float64)


def get_matrix_eular_radu(x,y,z,rx,ry,rz):
    rmat = tfs.euler.euler2mat(math.radians(rx),math.radians(ry),math.radians(rz),axes='sxyz')
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x,y,z))), rmat, [1, 1, 1])
    return rmat

def rot2quat_minimal(m):
    quat =  tfs.quaternions.mat2quat(m[0:3,0:3])
    return quat[1:]

def quatMinimal2rot(q):
    p = np.dot(q.T,q)
    w = np.sqrt(np.subtract(1,p[0][0]))
    return tfs.quaternions.quat2mat([w,q[0],q[1],q[2]])


def cal_camera_base(pose_hand,pose_camera):
    Hgs,Hcs = [],[]

    for i in range(len(pose_hand)):
        # if(pose_hand[i][3]>180):
        #     pose_hand[i][3] = pose_hand[i][3]-360
        Hgs.append(get_matrix_eular_radu(pose_hand[i][0],pose_hand[i][1],pose_hand[i][2],pose_hand[i][3],pose_hand[i][4],pose_hand[i][5]))    
        Hcs.append(get_matrix_eular_radu(pose_camera[i][0],pose_camera[i][1],pose_camera[i][2],pose_camera[i][3],pose_camera[i][4],pose_camera[i][5]))
    
    
    Hgijs=[]
    Hcijs=[]
    A = []
    B = []
    size = 0

    for i in range(len(Hgs)):
        for j in range(i+1,len(Hgs)):
            size += 1

            # Hgij = np.dot(np.linalg.inv(Hgs[j]),Hgs[i])
            Hgij = np.dot(Hgs[j],np.linalg.inv(Hgs[i]))

            Hgijs.append(Hgij)
            Pgij = np.dot(2,rot2quat_minimal(Hgij))

            Hcij = np.dot(Hcs[j],np.linalg.inv(Hcs[i]))
            # Hcij = np.dot(np.linalg.inv(Hcs[j]),Hcs[i])

            Hcijs.append(Hcij)
            Pcij = np.dot(2,rot2quat_minimal(Hcij))
            A.append(skew(np.add(Pgij,Pcij)))
            B.append(np.subtract(Pcij,Pgij))
    
    Hcg=np.zeros((4, 4), dtype=np.float64)
    Hcg=tsai_hand_eye(Hcg, Hgijs, Hcijs)
    np.set_printoptions(suppress=True)  # suppress参数用于禁用科学计数法
    print("方法三RT",Hcg)


    MA = np.asarray(A).reshape(size*3,3)
    MB = np.asarray(B).reshape(size*3,1)
    Pcg_ = np.dot(np.linalg.pinv(MA),MB)
    pcg_norm = np.dot(np.conjugate(Pcg_).T,Pcg_)
    Pcg = np.sqrt(np.add(1,np.dot(Pcg_.T,Pcg_)))
    Pcg = np.dot(np.dot(2,Pcg_),np.linalg.inv(Pcg))
    Rcg = quatMinimal2rot(np.divide(Pcg,2)).reshape(3,3)

    A = []
    B = []
    id = 0
    for i in range(len(Hgs)):
        for j in range(i+1,len(Hgs)):
            Hgij = Hgijs[id]
            Hcij = Hcijs[id]
            A.append(np.subtract(Hgij[0:3,0:3],np.eye(3,3)))
            B.append(np.subtract(np.dot(Rcg,Hcij[0:3,3:4]),Hgij[0:3,3:4]))
            id += 1

    MA = np.asarray(A).reshape(size*3,3)
    MB = np.asarray(B).reshape(size*3,1)
    Tcg = np.dot(np.linalg.pinv(MA),MB).reshape(3,)
    np.set_printoptions(suppress=True)  # suppress参数用于禁用科学计数法
    R_T=tfs.affines.compose(Tcg,np.squeeze(Rcg),[1,1,1])
    
    print("方法二RT",R_T)
    rot_matrix = np.array(R_T[:3,:3])
    # 创建Rotation对象  
    rot = R.from_matrix(rot_matrix)  
    # 获取ZYX顺序的欧拉角（以度为单位）  
    euler_angles_deg = rot.as_euler('xyz', degrees=True)  
    # 打印欧拉角  
    print("新欧拉角xyz(度):", euler_angles_deg)


if __name__ == "__main__":
    pose_camera=[[147.84474140749828, 66.54936701226228, 611.7865121623396, -4.208924466420011, 29.72409753162325, -39.78583873498079], [73.02129924622108, 15.761110101847974, 531.9567239984942, -6.214502200513373, 24.12792963318015, -25.979225986597147], [162.98447100326143, 104.92058675714752, 602.1196593477896, -5.471591438284376, 19.333733526995033, -43.489622849220034], [224.9993421748563, 117.13906350611957, 663.8546450968263, 0.9502399315726044, 5.848154551356732, -59.26591902287526], [102.56415158901524, 68.53004092822879, 592.5701134434745, -5.782899958004045, 24.547658890724595, -37.93166664415108], [162.13324320526894, 140.53374341122733, 630.3965347186518, -6.498357158221598, 11.013500417199184, -53.830242533568885], [80.03395385608658, 125.5842605529039, 631.3211218533986, -6.539636162638159, 27.620841567952773, -47.511516552403506], [-17.765626559793542, 193.3679002134944, 585.6922862756167, -24.334653642176548, 21.41383755318036, -41.979482257210464], [184.6517885575763, 99.27147142784652, 567.0557363037219, -5.452770904002674, 18.550345261359137, -36.0018200414512], [147.11253071255567, 109.0671596938967, 653.0685603292449, 2.9512155960845106, 27.93254620897932, -50.2210697731985], [-46.49942749434306, 162.1494873355198, 580.9546247108863, -29.697368386147083, 22.11201073742194, -41.44914819992306], [-63.464898819948466, 152.454112657339, 569.6124710147307, -31.82702827345418, 12.541064907707627, -40.855437029813196], [197.73013926588027, 50.602695230655, 495.99778768558866, -6.908697401047148, 19.46398964116439, -22.542713718828804], [9.83805691170268, -84.37023913527092, 512.3113194058143, -17.640175369179655, 21.184820268737766, -23.27969774934914], [16.791299089901884, 106.69803531708101, 596.7215684061342, -17.050343905792474, 27.808465977615935, -39.295360142293]]
    pose_hand=[[476.90838623046875, -157.9563751220703, -207.17054748535156, 123.6982866456551, -35.92194816156942, 78.5019885789037], [457.14129638671875, -93.80294036865236, -154.74362182617188, 134.18330722263576, -16.38444155169245, 82.27979342651206], [450.9874572753906, -173.14463806152344, -245.34921264648438, -179.21011971695586, 14.548252587332803, 83.62741925164454], [478.9930114746094, -220.43878173828128, -262.40301513671875, -141.4466238455358, 52.642530155868755, 68.52011562944084], [467.31744384765625, -120.02759552001955, -208.16477966308594, 140.62341405489423, -19.266467931918218, 82.68208094863708], [451.57122802734375, -175.0383071899414, -269.1492919921875, -126.94626479151223, 65.03160950512813, 77.41646348688515], [464.9386291503906, -97.85573005676271, -243.2484588623047, 134.46769229761506, -28.02901557441837, 83.89532792744444], [428.6229553222656, -32.31499099731447, -283.8015441894531, 17.984861847941705, -153.69479457401502, 79.23174552914107], [432.8953552246094, -193.74103546142578, -254.76683044433594, 171.46137902240895, 0.027558041499989357, 83.30563237361993], [480.44134521484375, -143.77264404296875, -261.2537841796875, 146.4573051903312, -22.338605747029955, 75.10600155433954], [452.19329833984375, -11.93625640869142, -233.51785278320312, -9.896694734737432, -173.78134162241363, 75.06423501403016], [468.688232421875, -3.4434280395507955, -275.7240295410156, 27.1851737472989, -157.3989369069371, 71.72549978171622], [418.4050598144531, -200.81698608398438, -201.97996520996094, -177.79325277916394, 20.099366188065403, 85.51883540120551], [505.603271484375, -55.922367095947266, -33.48153305053711, -59.402805535488035, 162.76890272435742, 83.56191883814215], [461.2748718261719, -53.629065513610854, -199.50157165527344, 11.507269325443957, -142.76321902273526, 85.25242840176757]]

   
    cal_camera_base(pose_hand,pose_camera)
