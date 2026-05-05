r25 = 10000
T25 = 298.15
B = 4200
low_t = 10
high_t = 150

vref = 3.3
adc_res = 4096

print("const uint16_t ntc_adc_table[] = {")
for T in range(low_t + 273, high_t + 273):
    resistance = r25 * pow(2.718, B*(1/T - 1/T25))

    adc_val = 4096 * 3300 / (resistance + 3300)

    #print("a", T, "gradi:", resistance, "ohm", "adc:", round(adc_val, 0))
    print("\t" + str(int(adc_val)) + ",")

print("}")