"""Replay recorded states with the actual C K; never connects to a serial port.

Local leg signs were checked by forward-swing tests. Outputs are diagnostic
projections only, not approved motor commands or a closed-loop validation.
"""
from pathlib import Path
import csv
import math
import re
import subprocess
import tempfile

software = Path(__file__).resolve().parents[1]
log = software / "tools/gravity_leg_test_20261018.log"
text = log.read_text(encoding="utf8")
active = text[text.index("SPRING active:"):]
active = active[:active.index("all motor targets set to 0 V")]
records = []
state = None
for line in active.splitlines():
    if line.startswith("STATE candidate:"):
        state = [float(v) for v in re.findall(r"(?:theta|dtheta|x|dx|phi|dphi)=([-\d.]+)", line)]
    elif line.startswith("STATE legs:") and state is not None:
        legs = re.findall(r"L=([-\d.]+) mm angle=([-\d.]+) deg rate=([-\d.]+) rad/s", line)
        lengths = [float(v[0])/1000 for v in legs]
        if len(state) == 6 and len(lengths) == 2 and all(.085 <= v <= .14 for v in lengths):
            angles = [math.radians(float(v[1])) for v in legs]
            rates = [float(v[2]) for v in legs]
            records.append((sum(lengths)/2, state, angles, rates))
        state = None
assert records, "No recorded samples in the working leg-length range"
reference = records[0][1][2]

def model_axes(s, angles, rates):
    # Historical log has forward-positive body pitch and an incorrect mean leg
    # angle. Rebuild theta from right/left local values; do not just flip it.
    return [(angles[0]-angles[1])/2+s[4], (rates[0]-rates[1])/2+s[5],
            s[2], s[3], -s[4], -s[5]]

inputs = []
for length, state, angles, rates in records:
    error = state.copy()
    error[2] -= reference
    inputs.extend([(length, error), (length, model_axes(error, angles, rates))])
# Static body-forward tilt with a locally vertical leg; diagnostic sign fixture only.
phi = math.radians(5)
fixture = [-phi, 0, 0, 0, phi, 0]
inputs.extend([(.1282, [0]*6), (.1282, fixture),
               (.1282, model_axes(fixture, [math.pi/2]*2, [0]*2))])
with tempfile.TemporaryDirectory(prefix="lqr-replay-") as directory:
    root = Path(directory)
    (root / "run.c").write_text("""#include <stdio.h>
#include "lqr_k.h"
int main(void) {
    float length,s[6],k[12];
    while(scanf("%f%f%f%f%f%f%f",&length,&s[0],&s[1],&s[2],&s[3],&s[4],&s[5])==7) {
        lqr_k(length,k); float t=0,tp=0;
        for(int i=0;i<6;i++) { t-=k[2*i]*s[i]; tp+=k[2*i+1]*s[i]; }
        printf("%.9g %.9g\\n",t,tp);
    }
}
""")
    exe = root / "run.exe"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I"+str(software / "main"), str(root / "run.c"),
                    str(software / "main/lqr_k.c"), "-o", str(exe)], check=True)
    data = "".join(" ".join(map(str, [length, *state]))+"\n" for length, state in inputs)
    output = subprocess.check_output([str(exe)], input=data, text=True)
    results = [tuple(map(float, line.split())) for line in output.splitlines()]
assert len(results) == len(inputs) and all(math.isfinite(x) for row in results for x in row)
assert all(abs(x)<1e-8 for x in results[-3]), "Upright equilibrium must be zero"
assert results[-2][0] < 0 < results[-1][0], "The two axis conventions should expose opposite wheel commands"
path = software / "tools/lqr_replay_20261018.csv"
with path.open("w", newline="", encoding="utf8") as f:
    writer=csv.writer(f)
    writer.writerow(["sample", "leg_length_m", "old_theta_rad", "old_dtheta_rad_s", "relative_x_m", "dx_m_s", "old_phi_rad", "old_dphi_rad_s", "original_formula_T_Nm", "original_formula_Tp_Nm", "model_axes_T_Nm", "model_axes_Tp_Nm", "model_T_limited_003Nm"])
    for i,(length,state,angles,rates) in enumerate(records):
        error=inputs[2*i][1]
        a,b=results[2*i:2*i+2]
        writer.writerow([i,length,*error,*a,*b,max(-.03,min(.03,b[0]))])
print(f"Replay: {len(records)} working-length samples; position reference {reference:.5f} m")
print("No serial commands or motor outputs were sent.")
print("5 deg body-forward fixture: original T,Tp=", results[-2], "model axes T,Tp=", results[-1])
print("Local leg signs confirmed separately; actuator coupling/contact still unvalidated.")
for name,index in [("original",0),("model",1)]:
    r=[results[2*i+index] for i in range(len(records))]
    print(name, "T range",min(x[0] for x in r),max(x[0] for x in r), "Tp range",min(x[1] for x in r),max(x[1] for x in r), "wheel torque clips",sum(abs(x[0])>.03 for x in r))
print(path)
