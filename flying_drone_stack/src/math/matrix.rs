//! Dense matrix types for the MEKF (9×9 covariance, 1×9 Jacobian rows).
//!
//! We use f32 throughout to match the Python implementation's numerical precision.
//! All arithmetic is straightforward; no BLAS/LAPACK dependency is needed for 9×9 matrices.

/// 9×9 matrix stored in row-major order, f32 precision.
#[derive(Debug, Clone, Copy)]
pub struct Mat9 {
    pub data: [[f32; 9]; 9],
}

impl Mat9 {
    /// Zero matrix.
    pub fn zeros() -> Self {
        Self { data: [[0.0; 9]; 9] }
    }

    /// Identity matrix.
    pub fn identity() -> Self {
        let mut m = Self::zeros();
        for i in 0..9 {
            m.data[i][i] = 1.0;
        }
        m
    }

    /// Diagonal matrix from a 9-element array.
    pub fn diag(d: [f32; 9]) -> Self {
        let mut m = Self::zeros();
        for i in 0..9 {
            m.data[i][i] = d[i];
        }
        m
    }

    /// Transpose.
    pub fn transpose(&self) -> Self {
        let mut out = Self::zeros();
        for i in 0..9 {
            for j in 0..9 {
                out.data[i][j] = self.data[j][i];
            }
        }
        out
    }

    /// Matrix–matrix multiply (self @ rhs).
    pub fn mat_mul(&self, rhs: &Mat9) -> Mat9 {
        let mut out = Mat9::zeros();
        for i in 0..9 {
            for j in 0..9 {
                let mut s = 0.0f32;
                for k in 0..9 {
                    s += self.data[i][k] * rhs.data[k][j];
                }
                out.data[i][j] = s;
            }
        }
        out
    }

    /// Matrix + Matrix (element-wise).
    pub fn add(&self, rhs: &Mat9) -> Mat9 {
        let mut out = Self::zeros();
        for i in 0..9 {
            for j in 0..9 {
                out.data[i][j] = self.data[i][j] + rhs.data[i][j];
            }
        }
        out
    }

    /// Scalar multiply.
    pub fn scale(&self, s: f32) -> Mat9 {
        let mut out = *self;
        for i in 0..9 {
            for j in 0..9 {
                out.data[i][j] *= s;
            }
        }
        out
    }

    /// Multiply matrix by a column vector: (9×9) × (9,) → (9,).
    pub fn mat_vec(&self, v: &[f32; 9]) -> [f32; 9] {
        let mut out = [0.0f32; 9];
        for i in 0..9 {
            for j in 0..9 {
                out[i] += self.data[i][j] * v[j];
            }
        }
        out
    }

    /// Outer product of two 9-vectors: u ⊗ v  →  9×9 matrix.
    pub fn outer(u: &[f32; 9], v: &[f32; 9]) -> Mat9 {
        let mut out = Mat9::zeros();
        for i in 0..9 {
            for j in 0..9 {
                out.data[i][j] = u[i] * v[j];
            }
        }
        out
    }

    /// Full Joseph-form covariance update: (I-KH) Σ (I-KH)ᵀ + K·r·Kᵀ
    ///
    /// This symmetric form is numerically stable for f32: it keeps Σ
    /// positive-definite even after thousands of sequential updates.
    /// K: 9×1 Kalman gain, H: 1×9 measurement row, r: scalar measurement noise.
    pub fn joseph_update(sigma: &Mat9, k: &[f32; 9], h: &[f32; 9], r: f32) -> Mat9 {
        // A = I - K·H  (9×9)
        let kh = Mat9::outer(k, h);
        let mut a = Mat9::identity();
        for i in 0..9 {
            for j in 0..9 {
                a.data[i][j] -= kh.data[i][j];
            }
        }
        // (I-KH) Σ (I-KH)ᵀ
        let asat_t = a.mat_mul(sigma).mat_mul(&a.transpose());
        // K·r·Kᵀ  (outer product of K scaled by r)
        let krkt = Mat9::outer(k, k).scale(r);
        asat_t.add(&krkt)
    }

    /// Row–vector dot product with matrix column: H · Σ · Hᵀ (scalar).
    /// H is a 1×9 row.
    pub fn h_sigma_ht(h: &[f32; 9], sigma: &Mat9) -> f32 {
        // first: tmp = H · Σ  (1×9)
        let mut tmp = [0.0f32; 9];
        for j in 0..9 {
            for k in 0..9 {
                tmp[j] += h[k] * sigma.data[k][j];
            }
        }
        // then: tmp · Hᵀ = dot(tmp, H)
        let mut s = 0.0f32;
        for j in 0..9 {
            s += tmp[j] * h[j];
        }
        s
    }

    /// Σ · Hᵀ  (9×1 column, returned as array).
    pub fn sigma_ht(sigma: &Mat9, h: &[f32; 9]) -> [f32; 9] {
        let mut out = [0.0f32; 9];
        for i in 0..9 {
            for j in 0..9 {
                out[i] += sigma.data[i][j] * h[j];
            }
        }
        out
    }

    /// Force exact symmetry: Σ ← (Σ + Σᵀ) / 2.
    ///
    /// Call after every covariance propagation step to prevent f32 rounding
    /// from accumulating anti-symmetric error in the positive-definite matrix.
    /// This is standard practice in embedded EKF implementations.
    pub fn symmetrise(&mut self) {
        for i in 0..9 {
            for j in (i + 1)..9 {
                let avg = 0.5 * (self.data[i][j] + self.data[j][i]);
                self.data[i][j] = avg;
                self.data[j][i] = avg;
            }
        }
    }

    /// Clamp each diagonal entry to `max_var`, zeroing off-diagonal entries in
    /// the same row/column proportionally.  Mirrors the Crazyflie firmware
    /// MAX_COVARIANCE guard (kalman_core.c) that prevents unbounded variance
    /// growth in f32 when states are poorly observed.
    pub fn clamp_diagonal(&mut self, max_var: f32) {
        for i in 0..9 {
            if self.data[i][i] > max_var {
                let scale = max_var / self.data[i][i];
                for j in 0..9 {
                    self.data[i][j] *= scale;
                    self.data[j][i] *= scale;
                }
                self.data[i][i] = max_var; // fix double-scaling of diagonal
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32) -> bool { (a - b).abs() < 1e-5 }

    fn mat9_approx_eq(a: &Mat9, b: &Mat9) -> bool {
        for i in 0..9 {
            for j in 0..9 {
                if !approx_eq(a.data[i][j], b.data[i][j]) { return false; }
            }
        }
        true
    }

    #[test]
    fn zeros_is_all_zero() {
        let m = Mat9::zeros();
        for i in 0..9 { for j in 0..9 { assert_eq!(m.data[i][j], 0.0); } }
    }

    #[test]
    fn identity_diagonal_ones_offdiag_zero() {
        let m = Mat9::identity();
        for i in 0..9 {
            for j in 0..9 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_eq!(m.data[i][j], expected);
            }
        }
    }

    #[test]
    fn diag_sets_diagonal() {
        let d = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let m = Mat9::diag(d);
        for i in 0..9 {
            assert_eq!(m.data[i][i], d[i]);
            for j in 0..9 { if i != j { assert_eq!(m.data[i][j], 0.0); } }
        }
    }

    #[test]
    fn transpose_swaps_indices() {
        let mut m = Mat9::zeros();
        // Fill with distinct values: m[i][j] = i*9 + j
        for i in 0..9 { for j in 0..9 { m.data[i][j] = (i * 9 + j) as f32; } }
        let t = m.transpose();
        for i in 0..9 { for j in 0..9 { assert_eq!(t.data[i][j], m.data[j][i]); } }
    }

    #[test]
    fn transpose_of_transpose_is_identity() {
        let mut m = Mat9::zeros();
        for i in 0..9 { for j in 0..9 { m.data[i][j] = (i * 9 + j) as f32; } }
        assert!(mat9_approx_eq(&m.transpose().transpose(), &m));
    }

    #[test]
    fn mat_mul_identity_unchanged() {
        let mut a = Mat9::zeros();
        for i in 0..9 { for j in 0..9 { a.data[i][j] = (i * 9 + j) as f32; } }
        let result = a.mat_mul(&Mat9::identity());
        assert!(mat9_approx_eq(&result, &a));
    }

    #[test]
    fn mat_mul_known_2x2_block() {
        // Use top-left 2×2 sub-block: [[1,2],[3,4]] * [[5,6],[7,8]] = [[19,22],[43,50]]
        let mut a = Mat9::zeros();
        let mut b = Mat9::zeros();
        a.data[0][0] = 1.0; a.data[0][1] = 2.0;
        a.data[1][0] = 3.0; a.data[1][1] = 4.0;
        b.data[0][0] = 5.0; b.data[0][1] = 6.0;
        b.data[1][0] = 7.0; b.data[1][1] = 8.0;
        let c = a.mat_mul(&b);
        assert!(approx_eq(c.data[0][0], 19.0));
        assert!(approx_eq(c.data[0][1], 22.0));
        assert!(approx_eq(c.data[1][0], 43.0));
        assert!(approx_eq(c.data[1][1], 50.0));
    }

    #[test]
    fn add_elementwise() {
        let a = Mat9::diag([1.0; 9]);
        let b = Mat9::diag([2.0; 9]);
        let c = a.add(&b);
        for i in 0..9 {
            assert!(approx_eq(c.data[i][i], 3.0));
            for j in 0..9 { if i != j { assert!(approx_eq(c.data[i][j], 0.0)); } }
        }
    }

    #[test]
    fn scale_multiplies_all_entries() {
        let m = Mat9::identity().scale(3.0);
        for i in 0..9 {
            assert!(approx_eq(m.data[i][i], 3.0));
            for j in 0..9 { if i != j { assert!(approx_eq(m.data[i][j], 0.0)); } }
        }
    }

    #[test]
    fn mat_vec_identity_returns_input() {
        let v = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let result = Mat9::identity().mat_vec(&v);
        for i in 0..9 { assert!(approx_eq(result[i], v[i])); }
    }

    #[test]
    fn outer_product_correctness() {
        let u = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let v = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let m = Mat9::outer(&u, &v);
        // u⊗v should be 1 at [0][1], 0 everywhere else
        assert!(approx_eq(m.data[0][1], 1.0));
        assert!(approx_eq(m.data[0][0], 0.0));
        assert!(approx_eq(m.data[1][1], 0.0));
    }

    #[test]
    fn h_sigma_ht_with_identity_equals_h_dot_h() {
        // H·I·H^T = H·H^T = sum(H_i^2)
        let h = [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let result = Mat9::h_sigma_ht(&h, &Mat9::identity());
        assert!(approx_eq(result, 5.0)); // 1² + 2² = 5
    }

    #[test]
    fn sigma_ht_with_identity_returns_h() {
        let h = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0];
        let result = Mat9::sigma_ht(&Mat9::identity(), &h);
        assert!(approx_eq(result[0], 3.0));
        assert!(approx_eq(result[8], 4.0));
    }

    #[test]
    fn joseph_update_preserves_symmetry() {
        // K = e_0, H = e_0 (update first state), r = 1.0
        // Result should still be symmetric
        let sigma = Mat9::identity();
        let k = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let h = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let updated = Mat9::joseph_update(&sigma, &k, &h, 1.0);
        for i in 0..9 {
            for j in 0..9 {
                assert!(approx_eq(updated.data[i][j], updated.data[j][i]),
                    "Not symmetric at [{i}][{j}]");
            }
        }
    }

    #[test]
    fn symmetrise_corrects_asymmetry() {
        let mut m = Mat9::zeros();
        m.data[0][1] = 3.0; // asymmetric: [0][1]=3, [1][0]=0
        m.symmetrise();
        assert!(approx_eq(m.data[0][1], 1.5));
        assert!(approx_eq(m.data[1][0], 1.5));
    }

    #[test]
    fn clamp_diagonal_caps_large_variance() {
        let mut m = Mat9::identity().scale(10.0); // diagonal = 10
        m.clamp_diagonal(1.0);
        for i in 0..9 {
            assert!(m.data[i][i] <= 1.0 + 1e-5,
                "diagonal[{i}] = {} > 1.0", m.data[i][i]);
        }
    }

    #[test]
    fn clamp_diagonal_unchanged_when_below_max() {
        let mut m = Mat9::diag([0.5; 9]);
        m.clamp_diagonal(1.0);
        for i in 0..9 { assert!(approx_eq(m.data[i][i], 0.5)); }
    }
}
