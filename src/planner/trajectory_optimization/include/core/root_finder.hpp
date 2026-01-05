/*
    MIT License

    Copyright (c) 2020 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#pragma once


#ifndef ROOT_FINDER_HPP
#define ROOT_FINDER_HPP

#define USE_MATH_DEFINES
#include <Eigen/Eigen>
#include <cfloat>
#include <cmath>
#include <set>

namespace RootFinderParam {
    constexpr size_t HIGHEST_ORDER = 64;
}

namespace RootFinderPriv {
    /* 
    Modulus of u(x)/v(x)
    The leading coefficient of v, i.e., v[0], must be 1.0 or -1.0
    The length of u, v, and r are lu, lv, and lu, respectively
    */
    inline int poly_mod(double* u, double* v, double* r, int lu, int lv);

    // Evaluate the polynomial p(x), which has len coefficients
    // Note: Horner scheme should not be employed here !!!
    // Horner scheme has bad numerical stability despite of its efficiency.
    // These errors are particularly troublesome for root-finding algorithms.
    // When the polynomial is evaluated near a zero, catastrophic
    // cancellation (subtracting two nearby numbers) is guaranteed to occur.
    // Therefore, Horner scheme may slow down some root-finding algorithms.
    inline double poly_eval(double* p, int len, double x);

    // Calculate all roots of a*x^3 + b*x^2 + c*x + d = 0
    inline std::set<double> solve_cub(double a, double b, double c, double d);
    
    /*  Solve resolvent eqaution of corresponding Quartic equation
        The input x must be of length 3
        Number of zeros are returned
    */
     inline int solve_resolvent(double* x, double a, double b, double c);

    // Calculate all roots of the monic quartic equation:
    // x^4 + a*x^3 + b*x^2 + c*x +d = 0
    inline std::set<double>
    solve_quart_monic(double a, double b, double c, double d);


    // Calculate the quartic equation: a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
    // All coefficients can be zero
    inline std::set<double>
    solve_quart(double a, double b, double c, double d, double e);


    // Calculate roots of coeffs(x) inside (lbound, rbound) by computing eigen values of its companion matrix
    // Complex roots with magnitude of imaginary part less than tol are considered real
    inline std::set<double> eigen_solve_real_roots(const Eigen::VectorXd& coeffs,
        double lbound,
        double ubound,
        double tol);

    // Calculate the number of sign variations of the Sturm sequences at x
    // The i-th sequence with size szSeq[i] stored in sturmSeqs[i][], 0 <= i < len
    inline double num_sign_var(double x, double** sturmSeqs, int* szSeq, int len);


    // Calculate the derivative poly coefficients of a given poly
    inline void poly_deri(double* coeffs, double* dcoeffs, int len);


    // Safe Newton Method
    // Requirements: f(l)*f(h)<=0
    template <typename F, typename DF>
    inline double safe_newton(const F& func,
        const DF& dfunc,
        const double& l,
        const double& h,
        const double& tol,
        const int& maxIts)
    {
        double xh, xl;
        double fl = func(l);
        double fh = func(h);
        if (fl == 0.0) {
            return l;
        }
        if (fh == 0.0) {
            return h;
        }
        if (fl < 0.0) {
            xl = l;
            xh = h;
        }
        else {
            xh = l;
            xl = h;
        }

        double rts = 0.5 * (xl + xh);
        double dxold = fabs(xh - xl);
        double dx = dxold;
        double f = func(rts);
        double df = dfunc(rts);
        double temp;
        for (int j = 0; j < maxIts; j++) {
            if ((((rts - xh) * df - f) * ((rts - xl) * df - f) > 0.0) ||
                (fabs(2.0 * f) > fabs(dxold * df))) {
                dxold = dx;
                dx = 0.5 * (xh - xl);
                rts = xl + dx;
                if (xl == rts) {
                    break;
                }
            }
            else {
                dxold = dx;
                dx = f / df;
                temp = rts;
                rts -= dx;
                if (temp == rts) {
                    break;
                }
            }

            if (fabs(dx) < tol) {
                break;
            }

            f = func(rts);
            df = dfunc(rts);
            if (f < 0.0) {
                xl = rts;
            }
            else {
                xh = rts;
            }
        }

        return rts;
    }
        // Calculate a single zero of poly coeffs(x) inside [lbound, ubound]
    // Requirements: coeffs(lbound)*coeffs(ubound) < 0, lbound < ubound
    inline double shrink_interval(double* coeffs,
        int numCoeffs,
        double lbound,
        double ubound,
        double tol);
   
    // Isolate all roots of sturmSeqs[0](x) inside interval (l, r) recursively and store them in rts
    // Requirements: fl := sturmSeqs[0](l) != 0, fr := sturmSeqs[0](r) != 0, l < r,
    //               lnv != rnv, lnv = numSignVar(l), rnv = numSignVar(r)
    //               sturmSeqs[0](x) must have at least one root inside (l, r)
    inline void recur_isolate(double l,
        double r,
        double fl,
        double fr,
        int lnv,
        int rnv,
        double tol,
        double** sturmSeqs,
        int* szSeq,
        int len,
        std::set<double>& rts);

    // Calculate roots of coeffs(x) inside (lbound, rbound) leveraging Sturm theory
    // Requirement: leading coefficient must be nonzero
    //              coeffs(lbound) != 0, coeffs(rbound) != 0, lbound < rbound
    inline std::set<double> isolate_real_roots(const Eigen::VectorXd& coeffs,
        double lbound,
        double ubound,
        double tol);
}// namespace RootFinderPriv

namespace RootFinder {
    // Calculate the convolution of lCoef(x) and rCoef(x)
    inline Eigen::VectorXd poly_conv(const Eigen::VectorXd& lCoef,
        const Eigen::VectorXd& rCoef);


        
    // // This function needs FFTW 3 and only performs better when the scale is large
    // inline Eigen::VectorXd polyConvFFT(const Eigen::VectorXd &lCoef, const Eigen::VectorXd &rCoef)
    // // Calculate the convolution of lCoef(x) and rCoef(x) using FFT
    // // This function is fast when orders of both poly are larger than 100
    // {
    //     int paddedLen = lCoef.size() + rCoef.size() - 1;
    //     int complexLen = paddedLen / 2 + 1;
    //     Eigen::VectorXd result(paddedLen);
    //     double *rBuffer = fftw_alloc_real(paddedLen);
    //     // Construct FFT plan and buffers
    //     fftw_complex *cForwardBuffer = fftw_alloc_complex(complexLen);
    //     fftw_complex *cBackwardBuffer = fftw_alloc_complex(complexLen);
    //     fftw_plan forwardPlan = fftw_plan_dft_r2c_1d(paddedLen, rBuffer, cForwardBuffer,
    //                                                  FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
    //     fftw_plan backwardPlan = fftw_plan_dft_c2r_1d(paddedLen, cBackwardBuffer, rBuffer,
    //                                                   FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
    //     // Pad lCoef by zeros
    //     int len = lCoef.size();
    //     for (int i = 0; i < len; i++)
    //     {
    //         rBuffer[i] = lCoef(i);
    //     }
    //     for (int i = len; i < paddedLen; i++)
    //     {
    //         rBuffer[i] = 0.0;
    //     }
    //     // Compute fft(pad(lCoef(x)) and back it up
    //     fftw_execute(forwardPlan);
    //     memcpy(cBackwardBuffer, cForwardBuffer, sizeof(fftw_complex) * complexLen);
    //     // Pad rCoef by zeros
    //     len = rCoef.size();
    //     for (int i = 0; i < len; i++)
    //     {
    //         rBuffer[i] = rCoef(i);
    //     }
    //     for (int i = len; i < paddedLen; i++)
    //     {
    //         rBuffer[i] = 0.0;
    //     }
    //     // Compute fft(pad(rCoef(x))
    //     fftw_execute(forwardPlan);
    //     // Compute fft(pad(lCoef(x)).fft(pad(rCoef(x))
    //     double real, imag;
    //     for (int i = 0; i < complexLen; i++)
    //     {
    //         real = cBackwardBuffer[i][0];
    //         imag = cBackwardBuffer[i][1];
    //         cBackwardBuffer[i][0] = real * cForwardBuffer[i][0] -
    //                                 imag * cForwardBuffer[i][1];
    //         cBackwardBuffer[i][1] = imag * cForwardBuffer[i][0] +
    //                                 real * cForwardBuffer[i][1];
    //     }
    //     // Compute ifft(fft(pad(lCoef(x)).fft(pad(rCoef(x)))
    //     fftw_execute(backwardPlan);
    //     // Recover the original intensity
    //     double intensity = 1.0 / paddedLen;
    //     for (int i = 0; i < paddedLen; i++)
    //     {
    //         result(i) = rBuffer[i] * intensity;
    //     }
    //     // Destruct FFT plan and buffers
    //     fftw_destroy_plan(forwardPlan);
    //     fftw_destroy_plan(backwardPlan);
    //     fftw_free(rBuffer);
    //     fftw_free(cForwardBuffer);
    //     fftw_free(cBackwardBuffer);
    //     return result;
    // }

    // Calculate self-convolution of coef(x)
    inline Eigen::VectorXd poly_sqr(const Eigen::VectorXd& coef);

        // Evaluate the polynomial at x, i.e., coeffs(x)
    // Horner scheme is faster yet less stable
    // Stable one should be used when coeffs(x) is close to 0.0
    inline double poly_val(const Eigen::VectorXd& coeffs,
        double x,
        bool numericalStability = true);

    // Count the number of distinct roots of coeffs(x) inside (l, r), leveraging Sturm theory
    // Boundary values, i.e., coeffs(l) and coeffs(r), must be nonzero
    inline int count_roots(const Eigen::VectorXd& coeffs, double l, double r);

    // Calculate roots of coeffs(x) inside (lbound, rbound)
    //
    // Closed-form solutions are employed for reduced_order < 5
    // isolation = true:
    //                    Sturm' theory and some geometrical property are employed to bracket each root
    //                    Safe-Newton is employed to shrink the interval efficiently
    // isolation = false:
    //                    Eigen values of polynomial companion matrix are calculated
    //
    // Requirement: leading coefficient must be nonzero
    //              coeffs(lbound) != 0, coeffs(rbound) != 0, lbound < rbound
    inline std::set<double> solve_polynomial(const Eigen::VectorXd& coeffs,
        double lbound,
        double ubound,
        double tol,
        bool isolation = true);

}// namespace RootFinder

#endif