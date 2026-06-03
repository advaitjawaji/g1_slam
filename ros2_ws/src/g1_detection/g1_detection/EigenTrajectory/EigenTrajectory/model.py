import torch
import torch.nn as nn
from .anchor import ETAnchor
from .descriptor import ETDescriptor
import baseline as baseline_module  # Dynamic baseline loading

class EigenTrajectory(nn.Module):
    r"""Standalone EigenTrajectory model optimized for real-time inference."""

    def __init__(self, hyper_params):
        super().__init__()
        self.hyper_params = hyper_params
        
        # 1. Hyperparameters
        self.t_obs = hyper_params.obs_len
        self.t_pred = hyper_params.pred_len
        self.k = hyper_params.k
        self.s = hyper_params.num_samples
        self.dim = hyper_params.traj_dim
        self.static_dist = hyper_params.static_dist

        # 2. Standalone Baseline Initialization
        # Dynamically loads the baseline (e.g., social_stgcnn) from the baseline folder
        baseline_name = hyper_params.baseline
        base_lib = getattr(baseline_module, baseline_name)
        
        self.baseline_model = base_lib.TrajectoryPredictor(hyper_params)
        self.hook_func = {
            "model_forward_pre_hook": base_lib.model_forward_pre_hook,
            "model_forward": base_lib.model_forward,
            "model_forward_post_hook": base_lib.model_forward_post_hook
        }

        # 3. ET Specific Components
        self.ET_m_descriptor = ETDescriptor(hyper_params=hyper_params, norm_sca=True)
        self.ET_s_descriptor = ETDescriptor(hyper_params=hyper_params, norm_sca=False)
        self.ET_m_anchor = ETAnchor(hyper_params=hyper_params)
        self.ET_s_anchor = ETAnchor(hyper_params=hyper_params)

    @torch.no_grad()
    def forward(self, obs_traj, addl_info=None):
        """
        Pure inference pass. 
        Input: obs_traj [N_ped, 8, 2]
        Output: Dict with 'recon_traj' [Samples, N_ped, 12, 2]
        """
        n_ped = obs_traj.size(0)
        device = obs_traj.device

        # --- A. Motion Masking ---
        # Discards paths where displacement is below static_dist
        mask = (obs_traj[:, -1] - obs_traj[:, -3]).div(2).norm(p=2, dim=-1) > self.static_dist
        obs_m_traj = obs_traj[mask]
        obs_s_traj = obs_traj[~mask]

        # --- B. ET Space Projection ---
        # Converts [x, y] to low-rank coefficients (C_obs)
        C_m_obs, _ = self.ET_m_descriptor.projection(obs_m_traj)
        C_s_obs, _ = self.ET_s_descriptor.projection(obs_s_traj)
        
        C_obs = torch.zeros((self.k, n_ped), dtype=torch.float, device=device)
        C_obs[:, mask], C_obs[:, ~mask] = C_m_obs, C_s_obs

        # --- C. Scene Centering ---
        obs_m_ori = self.ET_m_descriptor.traj_normalizer.traj_ori.squeeze(dim=1).T
        obs_s_ori = self.ET_s_descriptor.traj_normalizer.traj_ori.squeeze(dim=1).T
        obs_ori = torch.zeros((2, n_ped), dtype=torch.float, device=device)
        obs_ori[:, mask], obs_ori[:, ~mask] = obs_m_ori, obs_s_ori
        obs_ori -= obs_ori.mean(dim=1, keepdim=True)

        # --- D. Unified Prediction Pass ---
        # Uses the hooks to talk to the internal baseline_model
        input_data = self.hook_func["model_forward_pre_hook"](C_obs, obs_ori, addl_info)
        output_data = self.hook_func["model_forward"](input_data, self.baseline_model)
        C_pred_refine = self.hook_func["model_forward_post_hook"](output_data, addl_info)

        # --- E. Final Reconstruction ---
        C_m_pred = self.ET_m_anchor(C_pred_refine[:, mask])
        C_s_pred = self.ET_s_anchor(C_pred_refine[:, ~mask])

        pred_m_traj_recon = self.ET_m_descriptor.reconstruction(C_m_pred)
        pred_s_traj_recon = self.ET_s_descriptor.reconstruction(C_s_pred)
        
        final_preds = torch.zeros((self.s, n_ped, self.t_pred, self.dim), 
                                   dtype=torch.float, device=device)
        final_preds[:, mask], final_preds[:, ~mask] = pred_m_traj_recon, pred_s_traj_recon

        return {"recon_traj": final_preds}