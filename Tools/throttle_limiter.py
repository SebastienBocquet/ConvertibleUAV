from pytest import approx

# K_B  = 0.9570225318069994
# K_A  = 1.0808402968188415

K_A  = 0.9570225318069994
K_B  = 1.0808402968188415

pwm_in = 4000


th_a = K_A * pwm_in
th_b = K_B * pwm_in
th_c = th_a

if th_a > 4000:
    err_a = 4000 - th_a
    corr_b = (th_a + err_a) * K_B / K_A - th_b
    corr_a = 0
    th_a = 4000
    th_b = th_b + corr_b
if th_b > 4000:
    err_b = 4000 - th_b
    corr_a = (th_b + err_b) * K_A / K_B - th_a
    print(corr_a)
    th_b = 4000
    th_a = th_a + corr_a
th_c = th_a

print("throttle", th_a, th_b, th_c)
print("mean throttle", (1.0 / 3) * (th_a + th_b + th_c))

assert th_b == approx(K_B / K_A * th_a, 1e-9)
assert th_c == approx(th_a, 1e-9)
assert th_a <= 4000 and th_a >= 2000
assert th_b <= 4000 and th_b >= 2000
assert th_c <= 4000 and th_c >= 2000
