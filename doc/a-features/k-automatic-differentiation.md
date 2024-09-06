\page md_doc_a-features_k-automatic-differentiation Automatic differentiation and source code generation

In addition to analytical derivatives, Pinocchio supports automatic
differentiation. This is made possible through the full *scalar*
templatization of the whole C++ code and the use of any external library
that does automatic differentiation: ADOL-C, CasADi, CppAD and others. It is
important to keep in mind that these automatic derivatives are often
much slower than the analytical ones.

Another unique but central feature of Pinocchio is its ability to
generate code both at compile time and at runtime. This is achieved by
using another external toolbox called CppADCodeGen built on top of
CppAD. From any function using Pinocchio, CppADCodeGen is
able to generate on the fly its code in various languages: C, Latex,
etc. and to make some simplifications of the math expressions. Thanks to
this procedure, a code tailored for a specific robot model can be
generated and used externally to Pinocchio.
