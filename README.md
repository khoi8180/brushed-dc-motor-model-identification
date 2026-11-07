# First-Order DC Motor Identification via Position Integration

**First-order DC motor system identification via kinematic position integration (two-point method)**

**Author:** Nguyen Huu Anh Khoi  
**GitHub:** [@khoi8180](https://github.com/khoi8180)  
**Contact:** [nguyenhuuanhkhoi@gmail.com](mailto:nguyenhuuanhkhoi@gmail.com)

---

The first-order transfer function of a DC motor from input voltage $`V(s)`$ to angular velocity $`\Omega(s)`$ is given by:

```math
\frac{\Omega(s)}{V(s)} = \frac{K_{\text{DC}}}{\tau s + 1}
```

Estimating angular velocity from discrete encoder counts typically relies on Backward Euler finite differences:

```math
\omega[k] = \frac{\theta[k] - \theta[k-1]}{T_s}
```

However, numerical differentiation acts as a high-pass filter that severely amplifies quantization noise. For a sampling period of $`T_s = 1\text{ ms} = 0.001\text{ s}`$, a single-count quantization discrepancy between consecutive intervals ($`\theta[k] - \theta[k-1]`$ vs. $`\theta[k+1] - \theta[k]`$) is magnified by a factor of 1000.

To eliminate differentiation noise entirely, we formulate the identification problem in the **position domain**. Using the kinematic relationship between angular velocity and position:

```math
\theta(t) = \int_0^t \omega(\sigma) \, d\sigma \implies \frac{\Theta(s)}{\Omega(s)} = \frac{1}{s}
```

the transfer function from input voltage to output shaft position becomes:

```math
\frac{\Theta(s)}{V(s)} = \frac{\Omega(s)}{V(s)} \cdot \frac{\Theta(s)}{\Omega(s)} = \frac{K_{\text{DC}}}{s(\tau s + 1)}
```

---

## Mathematical Derivation

Consider a step input voltage of amplitude $`V_0`$:

```math
V(t) = V_0 u(t), \quad u(t) = \begin{cases} 1, & t \ge 0 \\ 0, & t < 0 \end{cases} \implies V(s) = \frac{V_0}{s}
```

The Laplace transform of the position response is:

```math
F(s) = \Theta(s) = \frac{K_{\text{DC}} V_0}{s^2(\tau s + 1)}
```

By the Cauchy Residue Theorem, the inverse Laplace transform $`\theta(t) = \mathcal{L}^{-1}\{F(s)\}`$ is the sum of the residues of $`F(s)e^{st}`$ at its poles ($`s = 0`$ of multiplicity 2, and $`s = -1/\tau`$ of multiplicity 1):

1. **Residue at the second-order pole $`s = 0`$:**

```math
\text{Res}[F(s)e^{st}, s = 0] = \lim_{s \to 0} \frac{d}{ds}\left[ s^2 F(s)e^{st} \right] = \lim_{s \to 0} \frac{d}{ds}\left[ \frac{K_{\text{DC}} V_0 e^{st}}{\tau s + 1} \right]
```

```math
= \lim_{s \to 0} K_{\text{DC}} V_0 \frac{t e^{st}(\tau s + 1) - \tau e^{st}}{(\tau s + 1)^2} = K_{\text{DC}} V_0 (t - \tau)
```

2. **Residue at the simple pole $`s = -1/\tau`$:**

```math
\text{Res}\left[F(s)e^{st}, s = -\frac{1}{\tau}\right] = \lim_{s \to -1/\tau} \left(s + \frac{1}{\tau}\right) \frac{K_{\text{DC}} V_0 e^{st}}{\tau s^2 \left(s + \frac{1}{\tau}\right)} = \frac{K_{\text{DC}} V_0 e^{-t/\tau}}{\tau \left(-\frac{1}{\tau}\right)^2} = K_{\text{DC}} V_0 \tau e^{-\frac{t}{\tau}}
```

Summing the residues yields the exact time-domain position response:

```math
\theta(t) = K_{\text{DC}} V_0 (t - \tau) + K_{\text{DC}} V_0 \tau e^{-\frac{t}{\tau}} = K_{\text{DC}} V_0 t + K_{\text{DC}} V_0 \tau \left(e^{-\frac{t}{\tau}} - 1\right)
```

### Asymptotic Linear Ramp

For $`t \ge 5\tau`$, the exponential transient decays to zero ($`e^{-t/\tau} < 0.0067 \approx 0`$), and $`\theta(t)`$ converges to a linear asymptote:

```math
r(t) = \theta(t \gg \tau) = K_{\text{DC}} V_0 t - K_{\text{DC}} V_0 \tau = a t + b
```

where $`a`$ is the slope and $`b`$ is the intercept:

```math
a = K_{\text{DC}} V_0, \qquad b = -K_{\text{DC}} V_0 \tau
```

From $`a`$ and $`b`$, the motor parameters are recovered directly:

```math
K_{\text{DC}} = \frac{a}{V_0}, \qquad \tau = -\frac{b}{a}
```

---

## Experimental Procedure (Two-Point Method)

Because the steady-state response is a straight line, continuous high-frequency data logging and external system identification toolboxes are unnecessary. Only **two position measurements** taken after the transient has settled ($`t_2 > t_1 \ge 5\tau`$) are required.

1. Configure the encoder counter to accumulate pulses monotonically without resetting. The conversion from raw counts $`N(t)`$ to output shaft angle $`\theta(t)`$ in radians is:

```math
\text{CPR} = \text{Decoding Factor} \times \text{PPR} \times \text{Gear Ratio}
```

```math
\theta(t) = N(t) \cdot \frac{2\pi}{\text{CPR}} \text{ (rad)}
```

2. Apply a constant step input voltage $`V_0`$ at $`t = 0`$.
3. Record two position points in the steady-state linear region:
   * $`\theta_1 = \theta(t_1)`$ at time $`t_1`$ (e.g., $`t_1 = 1.0\text{ s}`$)
   * $`\theta_2 = \theta(t_2)`$ at time $`t_2`$ (e.g., $`t_2 = 1.5\text{ s}`$)
4. Compute the line parameters $`(a, b)`$ and extract $`(K_{\text{DC}}, \tau)`$:

```math
a = \frac{\theta_2 - \theta_1}{t_2 - t_1}, \qquad b = \theta_1 - a t_1
```

```math
K_{\text{DC}} = \frac{a}{V_0}, \qquad \tau = -\frac{b}{a}
```  
