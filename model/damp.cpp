#pragma once 
#include <torch/torch.h>
#include "temporal.h"
#include "spectral.h"
#include "cross_attention.h"

struct dampnet : torch::nn::Module {
  
  public:
    dampnet(int num_det_classes, int num_mat_classes) {
    
      temporal_branch = register_module("temporal_branch", Temporal());

      spectral_brnach = register_module("spectral_branch", Spectral());
      
      corss_attn = register_module("corss_attn", CrossBranchAttention(128, 8));

    


    }

}
