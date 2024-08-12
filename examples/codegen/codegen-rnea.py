from pycppad import (
    ADCG,
    CG,
    Independent,
    ADCGFun,
    CodeHandler,
    LanguageC,
    LangCDefaultVariableNameGenerator,
)
import pinocchio.cppadcg as cgpin
import pinocchio as pin
import numpy as np

pinmodel = pin.buildSampleModelHumanoidRandom()
model = cgpin.Model(pinmodel)
data = model.createData()

nq = model.nq
nv = model.nv

x = np.array([ADCG(CG(0.0))] * (nq + nv + nv))
x[:nq] = cgpin.neutral(model)
Independent(x)

y = cgpin.rnea(model, data, x[:nq], x[nq : nq + nv], x[nq + nv :])

fun = ADCGFun(x, y)

# /***************************************************************************
# *                        Generate the C source code
# **************************************************************************/

# /**
# * start the special steps for source code generation for a Jacobian
# */
handler = CodeHandler(50)

indVars = np.array([CG(1.0)] * (nq + nv + nv))
handler.makeVariables(indVars)

jac = fun.Jacobian(indVars)

langC = LanguageC("double", 3)
nameGen = LangCDefaultVariableNameGenerator("y", "x", "v", "array", "sarray")
code = handler.generateCode(langC, jac, nameGen, "source")
print(code)
