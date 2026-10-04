function x = chi2inv_core(p, k)
% CHI2INV_CORE  Chi-square inverse CDF without the Statistics Toolbox.
x = 2 * gammaincinv(p, k / 2);
end
