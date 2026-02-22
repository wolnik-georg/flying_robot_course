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
