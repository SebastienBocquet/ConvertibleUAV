from pytest import approx

sqrt_k = 1.1293781085572312

pwm_in = 4000


def limit(x, min_val, max_val):
    if x < min_val:
        return min_val
    elif x > max_val:
        return max_val
    else:
        return x


th_a = (3.0 / (2 + sqrt_k)) * pwm_in
th_b = (3 * sqrt_k) / (2 + sqrt_k) * pwm_in
th_c = th_a

th_a_lim = limit(th_a, 2000, 4000)
th_b_lim = limit(th_b, 2000, 4000)
th_c_lim = limit(th_c, 2000, 4000)

err_a = th_a_lim - th_a
err_b = th_b_lim - th_b
err_c = th_c_lim - th_c

corr_b = err_b
corr_a = (1.0 / sqrt_k) * (th_b + corr_b - sqrt_k * th_a)
corr_c = corr_a
print(th_b + corr_b - sqrt_k * th_a)
print(th_a)
print(corr_b)
print(th_b)

th_a_corr = th_a + corr_a
th_b_corr = th_b + corr_b
th_c_corr = th_c + corr_c

print("throttle", th_a, th_b, th_c)
print("mean throttle", (1.0 / 3) * (th_a + th_b + th_c))
print("throttle lim", th_a_lim, th_b_lim, th_c_lim)
print("throttle corr", th_a_corr, th_b_corr, th_c_corr)
print("mean throttle corr", (1.0 / 3) * (th_a_corr + th_b_corr + th_c_corr))

assert (1.0 / 3) * (th_a + th_b + th_c) == approx(pwm_in, 1e-9)
assert th_b == approx(sqrt_k * th_a, 1e-9)
assert th_c == approx(th_a, 1e-9)
assert th_b_corr == approx(sqrt_k * th_a_corr, 1e-9)
assert th_c_corr == approx(th_a_corr, 1e-9)
assert th_a_corr <= 4000 and th_a_corr >= 2000
assert th_b_corr <= 4000 and th_b_corr >= 2000
assert th_c_corr <= 4000 and th_c_corr >= 2000
