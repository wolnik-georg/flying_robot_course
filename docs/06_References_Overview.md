# References Overview

Status: ✅ = Read | 🔍 = Skimmed | ⬜ = Not yet read

## Core Papers
| Paper | Year | Status | Importance | Core Idea |
|-------|------|--------|------------|---------|
| Neural-Augmented INDI (Cobo-Briesewitz et al.) | 2026 | ✅ | Critical | Hybrid neural + INDI residual estimation |
| Aggregate Downwash (Gielis et al.) | 2023 | ✅ | Critical | Learned aggregation of multi-vehicle downwash forces |
| Flatness-Preserving Residual (Hsieh et al.) | 2026 | ✅ | Critical | Residual learning that preserves differential flatness (FBL) |
| Neural-Swarm2 (Shi, Hönig et al.) | 2022 | ✅ | Critical | Heterogeneous Deep Sets for residual interaction forces |
| SO(2)-Equivariant Downwash (Smith et al.) | 2023 | ✅ | High | Geometry-aware residual force model |

## Additional Important Papers
| Paper | Year | Status | Importance | Core Idea |
|-------|------|--------|------------|---------|
| Neural-Swarm (original) | 2020 | ✅ | High | First permutation-invariant residual force NN |
| ProxFly | 2024/25 | ✅ | High | Residual RL for close-proximity / downwash compensation |
| Cascaded INDI (Smeur et al.) | 2018 | 🔍 | High | Classic cascaded INDI for MAVs |
| Tal & Karaman INDI | 2018/21 | ✅ | High | High-performance INDI for aggressive flight |
| RSS 2025 Downwash Characterisation | 2025 | 🔍 | High | Experimental forces, moments and flow fields |
| L1 KNODE-DW MPC | 2025 | 🔍 | High | Learning-based MPC + adaptive control for 3-robot teams |

## Lower Priority
| Paper | Year | Status | Importance | Core Idea |
|-------|------|--------|------------|---------|
| KNODE-DW MPC (2024) | 2024 | ⬜ | Medium-High | Residual learning inside MPC |
| Docking with Learnt Downwash | 2023 | ⬜ | Medium | Practical vertical docking with residual models |
| NDP-NMPC | 2023 | ⬜ | Medium | Neural predictor + NMPC |
| Batra et al. End-to-end RL Swarms | 2021/22 | ⬜ | Medium | Multi-agent end-to-end RL on Crazyflie |

> **Note:** KNODE-DW MPC (2024, Chee et al.) and L1 KNODE-DW MPC (2025, Hsieh et al.) are both important for Method 6 (Light Learning-based MPC / residual-MPC) in the updated 7-method control strategy list — see `docs/01_Thesis_Project_Snapshot.md` §2.
