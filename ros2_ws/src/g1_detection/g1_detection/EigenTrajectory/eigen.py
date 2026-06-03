import os
import sys
import cv2
import torch
import torch.nn as nn
import numpy as np
from ultralytics import YOLO
from collections import defaultdict, deque

try:
    from EigenTrajectory.anchor import ETAnchor
    from EigenTrajectory.descriptor import ETDescriptor
    from utils.utils import get_exp_config
    import baseline as baseline_module
except ImportError:
    try:
        from anchor import ETAnchor
        from descriptor import ETDescriptor
        from utils import get_exp_config
        import baseline as baseline_module
    except ImportError as e:
        print("CRITICAL IMPORT ERROR. Run this from the ROOT folder.")
        sys.exit(1)

class EigenTrajectoryInference(nn.Module):
    def __init__(self, hyper_params):
        super().__init__()
        
        print(f"[Init] Overwriting k={hyper_params.k} to k=6 (from checkpoint analysis).")
        hyper_params.k = 6          # The checkpoint uses only 6 eigenvectors
        hyper_params.pred_len = 12  # The checkpoint predicts 12 frames (standard)
        hyper_params.obs_len = 8
        self.hyper_params = hyper_params
        
        self.k = 6
        self.s = hyper_params.num_samples
        self.t_obs = 8
        self.t_pred = 12
        self.dim = hyper_params.traj_dim
        self.static_dist = 0.001 

        # Baseline Loader
        if hasattr(baseline_module, hyper_params.baseline):
            base_lib = getattr(baseline_module, hyper_params.baseline)
        elif hasattr(baseline_module, 'social_stgcnn'): 
            base_lib = baseline_module.social_stgcnn
        else:
            raise ValueError(f"Baseline not found")

        # FORCED ARGUMENTS 
        stgcnn_args = {
            'n_stgcnn': 1,
            'n_txpcnn': 5,
            'input_feat': 1,
            'output_feat': 20,
            'seq_len': 8,
            'kernel_size': 3,
            # In EigenTrajectory, the baseline predicts 'k' coefficients, NOT frames.
            # So pred_seq_len MUST equal k.
            'pred_seq_len': 6 
        }

        try:
            self.baseline_model = base_lib.TrajectoryPredictor(**stgcnn_args)
        except TypeError:
            # Fallback
            backup = {'obs_len': 8, 'pred_len': 6, 'traj_dim': 1}
            self.baseline_model = base_lib.TrajectoryPredictor(**backup)

        self.hook_func = {
            "model_forward_pre_hook": base_lib.model_forward_pre_hook,
            "model_forward": base_lib.model_forward,
            "model_forward_post_hook": base_lib.model_forward_post_hook
        }

        # Initialize Descriptors (Now with k=6)
        self.ET_m_descriptor = ETDescriptor(hyper_params=hyper_params, norm_sca=True)
        self.ET_s_descriptor = ETDescriptor(hyper_params=hyper_params, norm_sca=False)
        self.ET_m_anchor = ETAnchor(hyper_params=hyper_params)
        self.ET_s_anchor = ETAnchor(hyper_params=hyper_params)

    @torch.no_grad()
    def forward(self, obs_traj):
        n_ped = obs_traj.size(0)
        device = obs_traj.device

        # DISABLE MASKING FOR DEBUG
        mask = torch.ones(n_ped, dtype=torch.bool, device=device)
        
        obs_m_traj = obs_traj[mask]

        # Projection
        C_m_obs, _ = self.ET_m_descriptor.projection(obs_m_traj)
        
        C_obs = torch.zeros((self.k, n_ped), dtype=torch.float, device=device)
        C_obs[:, mask] = C_m_obs

        # Scene Centering
        obs_m_ori = self.ET_m_descriptor.traj_normalizer.traj_ori.squeeze(dim=1).T
        obs_ori = torch.zeros((2, n_ped), dtype=torch.float, device=device)
        obs_ori[:, mask] = obs_m_ori
        obs_ori -= obs_ori.mean(dim=1, keepdim=True)

        # Inference
        input_data = self.hook_func["model_forward_pre_hook"](C_obs, obs_ori, None)
        output_data = self.hook_func["model_forward"](input_data, self.baseline_model)
        C_pred_refine = self.hook_func["model_forward_post_hook"](output_data, None)

        # Reconstruction (Returns [..., 12, 2])
        C_m_pred = self.ET_m_anchor(C_pred_refine[:, mask])
        pred_m_traj_recon = self.ET_m_descriptor.reconstruction(C_m_pred)
        
        # Final Assignment
        final_preds = torch.zeros((self.s, n_ped, self.t_pred, self.dim), dtype=torch.float, device=device)
        final_preds[:, mask] = pred_m_traj_recon

        return final_preds


# Uncomment for testing
# # --- 3. MAIN LOOP ---
# def main():
#     BASE_DIR = os.path.dirname(os.path.abspath(__file__))
#     CFG_PATH = os.path.join(BASE_DIR, "config", "eigentrajectory-stgcnn-eth.json")
#     WEIGHTS_PATH = os.path.join(BASE_DIR, "checkpoints", "model_best.pth")
    
#     device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
#     print(f"Running on: {device}")

#     # Load Config
#     hyper_params = get_exp_config(CFG_PATH)
#     predictor = EigenTrajectoryInference(hyper_params).to(device)
    
#     # Load Weights
#     checkpoint = torch.load(WEIGHTS_PATH, map_location=device)
#     state_dict = checkpoint.get('model_state_dict', checkpoint)
#     predictor.load_state_dict(state_dict)
#     predictor.eval()
#     print("Weights Loaded Successfully!")

#     # YOLO
#     try:
#         yolo = YOLO("yolo26n.pt")
#     except:
#         yolo = YOLO("yolo26n.pt")

#     cap = cv2.VideoCapture(0)
#     track_history = defaultdict(lambda: deque(maxlen=8))
    
#     print("Starting Camera... (Press Q to exit)")

#     while cap.isOpened():
#         ret, frame = cap.read()
#         if not ret: break
#         h, w, _ = frame.shape

#         results = yolo.track(frame, persist=True, classes=[0], tracker="bytetrack.yaml", verbose=False)
        
#         active_ids = []
#         obs_batch = []

#         if results[0].boxes.id is not None:
#             boxes = results[0].boxes.xyxy.cpu().numpy()
#             ids = results[0].boxes.id.int().cpu().tolist()

#             for box, tid in zip(boxes, ids):
#                 cx = (box[0] + box[2]) / 2 / w
#                 fy = box[3] / h
#                 track_history[tid].append([cx, fy])
                
#                 if len(track_history[tid]) == 8:
#                     active_ids.append(tid)
#                     obs_batch.append(list(track_history[tid]))

#             # Prediction
#             if len(obs_batch) > 0:
#                 obs_tensor = torch.tensor(obs_batch).float().to(device)
                
#                 try:
#                     preds = predictor(obs_tensor) 
#                     best_pred = preds[0].cpu().numpy() # [N, 12, 2]

#                     for i, tid in enumerate(active_ids):
#                         path = best_pred[i]
                        
#                         if i == 0: 
#                              print(f"\r[DEBUG] ID {tid} Pred end: ({path[-1][0]:.2f}, {path[-1][1]:.2f})", end="")

#                         for j in range(len(path) - 1):
#                             x1 = int(path[j][0] * w)
#                             y1 = int(path[j][1] * h)
#                             x2 = int(path[j+1][0] * w)
#                             y2 = int(path[j+1][1] * h)
                            
#                             # Valid coords only
#                             if 0 <= x1 < w and 0 <= y1 < h:
#                                 cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 4)
                        
#                         end_x = int(path[-1][0]*w)
#                         end_y = int(path[-1][1]*h)
#                         cv2.circle(frame, (end_x, end_y), 6, (0,255,0), -1)

#                 except Exception as e:
#                     print(f"\nPred Error: {e}")

#         cv2.imshow("Factory Safety Feed", frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'): break

#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()