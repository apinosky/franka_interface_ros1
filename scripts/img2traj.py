import numpy as np 
import torch 
from geomloss import SamplesLoss
from tqdm import tqdm 
import score_lqr_so


cuda = torch.device("cuda:0")
use_cuda = torch.cuda.is_available()
dtype = torch.cuda.FloatTensor if use_cuda else torch.FloatTensor


def sinkhorn_flow_coverage(_x0, pts, dt=0.05, num_steps=300, num_iters=1000):
    tgt_distr = torch.from_numpy(pts).to(cuda, dtype=torch.float32)

    dim = 2 

    _v0 = (0.5 - _x0) * 2.0 / (dt * num_steps)
    x0 = np.array([_x0[0], _x0[1], _v0[0], _v0[1]])
    u_traj = np.zeros((num_steps, 2)) 

    Q = np.diag([1.0, 1.0, 0.0001, 0.0001])
    R = np.eye(2) * 0.01
    R_inv = np.linalg.inv(R)
    traj_agent = score_lqr_so.lqr(dt)

    ot_loss = SamplesLoss("sinkhorn", p=1, blur=0.01)
    step_size = 0.05

    for iter in tqdm(range(num_iters)):
        x_traj_np = traj_agent.traj_sim(x0, u_traj)
        x_traj = torch.from_numpy(x_traj_np[:,:2]).to(device=cuda, dtype=torch.float32)
        x_traj.requires_grad = True

        loss_val = ot_loss(x_traj, tgt_distr)
        [a_traj] = torch.autograd.grad(loss_val, [x_traj])
        a_traj *= -1.0 * num_steps
        a_traj = a_traj.detach().cpu().numpy()

        v_traj = traj_agent.get_descent(x0, u_traj, a_traj, Q, R_inv)
        u_traj += step_size * v_traj

    x_traj_np = traj_agent.traj_sim(x0, u_traj)
    return x_traj_np


#################################################################################################


import cv2
import numpy as np


def img2pts(image_path):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    height, width = img.shape
    scale = 1.0 / width
    points = np.array([(x, height - 1 - y) for y in range(height) for x in range(width) if img[y, x] < 100])
    if points.size == 0:
        return np.empty((0, 2)), width*scale, height*scale
    offset = np.array([width*scale, height*scale]) / 2.0
    normalized_points = points * scale - offset

    rng = np.random.default_rng(11)
    img_pts = rng.permutation(normalized_points, axis=0)[:1000]
    pts0 = rng.uniform(
        low=np.min(normalized_points), 
        high=np.max(normalized_points), 
        size=(10000,2)
    )
    dists = np.min(np.linalg.norm(pts0 - img_pts[:,None], axis=-1), axis=0)
    pts = pts0[np.where(dists<=0.02)]

    return pts, width*scale, height*scale


#################################################################################################

import numpy as np 


def img2traj(img_path):
    pts, width, height = img2pts(img_path)
    dists2origin = np.linalg.norm(pts, axis=1)
    x0 = pts[np.argmin(dists2origin)]
    if pts.shape[0] == 0:
        done = True
        traj = np.array([x0])
    else:
        done = False
        traj = sinkhorn_flow_coverage(x0, pts)
    return x0, traj, pts, width, height, done