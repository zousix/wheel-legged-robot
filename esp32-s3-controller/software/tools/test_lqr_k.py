"""Compare compiled firmware K with the MATLAB polynomial, including matrix layout."""
from pathlib import Path
import math
import re
import subprocess
import sys
import tempfile

software = Path(__file__).resolve().parents[1]
matlab = software.parents[1] / 'matlab' / 'lqr_k.m'
expressions = re.findall(r'mt\d+ = \[(.*?)\];', matlab.read_text(encoding='utf8'))
source = Path(sys.argv[1]).resolve() if len(sys.argv)>1 else software / 'main' / 'lqr_k.c'
assert len(expressions) == 12
with tempfile.TemporaryDirectory(prefix='lqr-k-') as directory:
    root = Path(directory)
    (root / 'test.c').write_text('''#include <stdio.h>
#include "lqr_k.h"
int main(void) {
    for (int i=0;i<=100;i++) {
        float length=.04f+.001f*i, k[12];
        lqr_k(length,k);
        printf("%.9g",length);
        for(int j=0;j<12;j++) printf(" %.9g",k[j]);
        puts("");
    }
}
''')
    executable = root / 'test.exe'
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(software / 'main'), str(root / 'test.c'),
                    str(source), '-o', str(executable)], check=True)
    lines = subprocess.check_output([str(executable)], text=True).splitlines()
    assert len(lines) == 101
    worst = 0.0
    for line in lines:
        length, *actual = map(float, line.split())
        for index, expression in enumerate(expressions):
            expected = eval(expression.replace('.*', '*'), {'__builtins__': {}},
                            {'L0': length, 't2': length**2, 't3': length**3})
            assert math.isfinite(actual[index])
            error = abs(actual[index]-expected)
            worst = max(worst, error)
            assert error < 2e-6, (length, index, actual[index], expected)
    print(f'PASS: 101 leg lengths, 12 column-major K coefficients; max error {worst:.3g}')
