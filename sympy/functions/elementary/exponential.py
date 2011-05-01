from sympy.core import C, sympify
from sympy.core.add import Add
from sympy.core.function import Lambda, Function, expand_log, ArgumentIndexError
from sympy.core.cache import cacheit
from sympy.core.singleton import S
from sympy.core.symbol import Wild, Symbol, Dummy
from sympy.core.mul import Mul

from sympy.ntheory import multiplicity

# NOTE IMPORTANT
# The series expansion code in this file is an important part of the gruntz
# algorithm for determining limits. _eval_nseries has to return a generalised
# power series with coefficients in C(log(x), log).
# In more detail, the result of _eval_nseries(self, x, n) must be
#   c_0*x**e_0 + ... (finitely many terms)
# where e_i are numbers (not necessarily integers) and c_i involve only
# numbers, the function log, and log(x). [This also means it must not contain
# log(x(1+p)), this *has* to be expanded to log(x)+log(1+p) if x.is_positive and
# p.is_positive.]

class exp(Function):

    nargs = 1

    def fdiff(self, argindex=1):
        if argindex == 1:
            return self
        else:
            raise ArgumentIndexError(self, argindex)

    def inverse(self, argindex=1):
        return log

    def as_numer_denom(self):
        c, t = self.args[0].as_coeff_mul()
        if c.is_negative:
            return S.One, exp(-self.args[0])
        return self, S.One

    @classmethod
    def eval(cls, arg):
        if arg.is_Number:
            if arg is S.NaN:
                return S.NaN
            elif arg is S.Zero:
                return S.One
            elif arg is S.One:
                return S.Exp1
            elif arg is S.Infinity:
                return S.Infinity
            elif arg is S.NegativeInfinity:
                return S.Zero
        elif arg.func is log:
            return arg.args[0]
        elif arg.is_Mul:
            coeff = arg.as_coefficient(S.Pi*S.ImaginaryUnit)

            if coeff is not None:
                if (2*coeff).is_integer:
                    if coeff.is_even:
                        return S.One
                    elif coeff.is_odd:
                        return S.NegativeOne
                    elif (coeff + S.Half).is_even:
                        return -S.ImaginaryUnit
                    elif (coeff + S.Half).is_odd:
                        return S.ImaginaryUnit
            I = S.ImaginaryUnit
            oo = S.Infinity
            a = Wild("a", exclude=[I, oo])
            r = arg.match(I*a*oo)
            if r and r[a] != 0:
                return S.NaN

        args = Add.make_args(arg)

        included, excluded = [], []

        for arg in args:
            coeff, terms = arg.as_coeff_mul()

            if coeff is S.Infinity:
                excluded.append(coeff**Mul(*terms))
            else:
                coeffs, log_term = [coeff], None

                for term in terms:
                    if term.func is log:
                        if log_term is None:
                            log_term = term.args[0]
                        else:
                            log_term = None
                            break
                    elif term.is_comparable:
                        coeffs.append(term)
                    else:
                        log_term = None
                        break

                if log_term is not None:
                    excluded.append(log_term**Mul(*coeffs))
                else:
                    included.append(arg)

        if excluded:
            return Mul(*(excluded + [cls(Add(*included))]))

    @property
    def base(self):
        return S.Exp1

    @property
    def exp(self):
        return self.args[0]

    @staticmethod
    @cacheit
    def taylor_term(n, x, *previous_terms):
        if n<0: return S.Zero
        if n==0: return S.One
        x = sympify(x)
        if previous_terms:
            p = previous_terms[-1]
            if p is not None:
                return p * x / n
        return x**n/C.Factorial()(n)

    def _eval_expand_complex(self, deep=True, **hints):
        re_part, im_part = self.as_real_imag(deep=deep, **hints)
        return re_part + im_part*S.ImaginaryUnit

    def as_real_imag(self, deep=True, **hints):
        re, im = self.args[0].as_real_imag()
        if deep:
            re = re.expand(deep, **hints)
            im = im.expand(deep, **hints)
        cos, sin = C.cos(im), C.sin(im)
        return (exp(re)*cos, exp(re)*sin)

    def _eval_conjugate(self):
        return self.func(self.args[0].conjugate())

    def as_base_exp(self):
        return S.Exp1, Mul(*self.args)

    def _eval_subs(self, old, new):
        if self == old:
            return new
        arg = self.args[0]
        o = old
        if old.is_Pow: # handle (exp(3*log(x))).subs(x**2, z) -> z**(3/2)
            old = exp(old.exp*log(old.base))
        if old.func is exp:
            # exp(a*expr) .subs( exp(b*expr), y )  ->  y ** (a/b)
            a, expr_terms = self.args[0].as_coeff_mul()
            b, expr_terms_= old.args[0].as_coeff_mul()

            if expr_terms == expr_terms_:
                return new**(a/b)


            if arg.is_Add: # exp(2*x+a).subs(exp(3*x),y) -> y**(2/3) * exp(a)
                # exp(exp(x) + exp(x**2)).subs(exp(exp(x)), w) -> w * exp(exp(x**2))
                oarg = old.args[0]
                new_l = []
                old_al = []
                coeff2, terms2 = oarg.as_coeff_mul()
                for a in arg.args:
                    a = a._eval_subs(old, new)
                    coeff1, terms1 = a.as_coeff_mul()
                    if terms1 == terms2:
                        new_l.append(new**(coeff1/coeff2))
                    else:
                        old_al.append(a._eval_subs(old, new))
                if new_l:
                    new_l.append(self.func(Add(*old_al)))
                    r = Mul(*new_l)
                    return r
        if old is S.Exp1:
            # treat this however Pow is being treated
            u = C.Dummy('u')
            return (u**self.args[0]).subs(u, new)

        old = o
        return Function._eval_subs(self, old, new)

    def _eval_is_real(self):
        return self.args[0].is_real

    def _eval_is_positive(self):
        if self.args[0].is_real:
            return True

    def _eval_is_bounded(self):
        arg = self.args[0]
        if arg.is_unbounded:
            if arg.is_negative: return True
            if arg.is_positive: return False
        if arg.is_bounded:
            return True
    def _eval_is_zero(self):
        return (self.args[0] is S.NegativeInfinity)

    def _eval_power(b, e):
        """exp(b[0])**e -> exp(b[0]*e)"""
        return exp(b.args[0] * e)

    def _eval_lseries(self, x):
        s = self.args[0]
        yield exp(s.subs(x, 0))
        from sympy import integrate
        t = Dummy("t")
        f = s.subs(x, t)
        for term in (exp(f)*f.diff(t)).lseries(t):
            yield integrate(term, (t, 0, x))

    def _eval_nseries(self, x, n):
        # NOTE Please see the comment at the beginning of this file, labelled
        #      IMPORTANT.
        from sympy import limit, oo, powsimp, expand_mul
        arg = self.args[0]
        arg_series = arg._eval_nseries(x, n=n)
        if arg_series.is_Order:
            return 1 + arg_series
        logx = Dummy('l')
        arg_series = arg_series.subs(log(x), logx)
        arg0 = limit(arg_series.removeO(), x, 0)
        if arg0 in [-oo, oo]:
            return self.subs(logx, log(x))
        t = Dummy("t")
        exp_series = exp(t)._taylor(t, n)
        o = exp_series.getO()
        exp_series = exp_series.removeO()
        r = exp(arg0)*exp_series.subs(t, arg_series - arg0)
        r = r.subs(logx, log(x))
        r += C.Order(o.expr.subs(t, (arg_series - arg0).subs(logx, log(x))), x)
        r = r.expand()
        return powsimp(r, deep=True, combine='exp')

    def _taylor(self, x, n):
        l = []
        g = None
        for i in xrange(n):
            g = self.taylor_term(i, self.args[0], g)
            g = g.nseries(x, n=n)
            l.append(g)
        return Add(*l) + C.Order(x**n, x)

    def _eval_as_leading_term(self, x):
        arg = self.args[0]
        if arg.is_Add:
            return Mul(*[exp(f).as_leading_term(x) for f in arg.args])
        arg = self.args[0].as_leading_term(x)
        if C.Order(1,x).contains(arg):
            return S.One
        return exp(arg)

    def _eval_expand_power_exp(self, deep=True, **hints):
        if deep:
            arg = self.args[0].expand(deep=deep, **hints)
        else:
            arg = self.args[0]
        if arg.is_Add and arg.is_commutative:
            expr = 1
            for x in arg.args:
                if deep:
                    x = x.expand(deep=deep, **hints)
                expr *= self.func(x)
            return expr
        return self.func(arg)

    def _eval_rewrite_as_sin(self, arg):
        I = S.ImaginaryUnit
        return C.sin(I*arg+S.Pi/2) - I*C.sin(I*arg)

    def _eval_rewrite_as_cos(self, arg):
        I = S.ImaginaryUnit
        return C.cos(I*arg) + I*C.cos(I*arg+S.Pi/2)

    def _sage_(self):
        import sage.all as sage
        return sage.exp(self.args[0]._sage_())

class log(Function):

    nargs = (1,2)

    def fdiff(self, argindex=1):
        if argindex == 1:
            return 1/self.args[0]
            s = C.Dummy('x')
            return Lambda(s**(-1), s)
        else:
            raise ArgumentIndexError(self, argindex)

    def inverse(self, argindex=1):
        return exp

    @classmethod
    def eval(cls, arg, base=None):
        if base is not None:
            base = sympify(base)

            if arg.is_positive and arg.is_Integer and \
               base.is_positive and base.is_Integer:
                base = int(base)
                arg = int(arg)
                n = multiplicity(base, arg)
                return S(n) + log(arg // base ** n) / log(base)
            if base is not S.Exp1:
                return cls(arg)/cls(base)
            else:
                return cls(arg)

        arg = sympify(arg)

        if arg.is_Number:
            if arg is S.Zero:
                return S.ComplexInfinity
            elif arg is S.One:
                return S.Zero
            elif arg is S.Infinity:
                return S.Infinity
            elif arg is S.NegativeInfinity:
                return S.Infinity
            elif arg is S.NaN:
                return S.NaN
            elif arg.is_negative:
                return S.Pi * S.ImaginaryUnit + cls(-arg)
        elif arg is S.ComplexInfinity:
            return S.ComplexInfinity
        elif arg is S.Exp1:
            return S.One
        #this doesn't work due to caching: :(
        #elif arg.func is exp and arg.args[0].is_real:
        #using this one instead:
        elif arg.func is exp and arg.args[0].is_real:
            return arg.args[0]
        #this shouldn't happen automatically (see the issue 252):
        #elif arg.is_Pow:
        #    if arg.exp.is_Number or arg.exp.is_NumberSymbol or \
        #        arg.exp.is_number:
        #        return arg.exp * self(arg.base)
        #elif arg.is_Mul and arg.is_real:
        #    return Add(*[self(a) for a in arg])
        elif not arg.is_Add:
            coeff = arg.as_coefficient(S.ImaginaryUnit)

            if coeff is not None:
                if coeff is S.Infinity:
                    return S.Infinity
                elif coeff is S.NegativeInfinity:
                    return S.Infinity
                elif coeff.is_Rational:
                    if coeff.is_nonnegative:
                        return S.Pi * S.ImaginaryUnit * S.Half + cls(coeff)
                    else:
                        return -S.Pi * S.ImaginaryUnit * S.Half + cls(-coeff)

    def as_base_exp(self):
        return self, S.One

    @staticmethod
    @cacheit
    def taylor_term(n, x, *previous_terms): # of log(1+x)
        from sympy import powsimp
        if n<0: return S.Zero
        x = sympify(x)
        if n==0: return x
        if previous_terms:
            p = previous_terms[-1]
            if p is not None:
                return powsimp((-n) * p * x / (n+1), deep=True, combine='exp')
        return (1-2*(n%2)) * x**(n+1)/(n+1)

    def _eval_expand_log(self, deep=True, **hints):
        if deep:
            arg = self.args[0].expand(deep=deep, **hints)
        else:
            arg = self.args[0]
        if arg.is_Mul:
            expr = sympify(0)
            nonpos = sympify(1)
            for x in arg.args:
                if deep:
                    x = x.expand(deep=deep, **hints)
                if x.is_positive:
                    expr += self.func(x)._eval_expand_log(deep=deep, **hints)
                else:
                    nonpos *= x
            return expr + log(nonpos)
        elif arg.is_Pow:
            if arg.exp.is_real and arg.base.is_positive:
                if deep:
                    b = arg.base.expand(deep=deep, **hints)
                    e = arg.exp.expand(deep=deep, **hints)
                else:
                    b = arg.base
                    e = arg.exp
                return e * self.func(b)._eval_expand_log(deep=deep,\
                **hints)
        return self.func(arg)

    def as_real_imag(self, deep=True, **hints):
        if deep:
            abs = C.Abs(self.args[0].expand(deep, **hints))
            arg = C.arg(self.args[0].expand(deep, **hints))
        else:
            abs = C.Abs(self.args[0])
            arg = C.arg(self.args[0])
        if hints.get('log', False): # Expand the log
            hints['complex'] = False
            return (log(abs).expand(deep, **hints), arg)
        else:
            return (log(abs), arg)

    def _eval_expand_complex(self, deep=True, **hints):
        re_part, im_part = self.as_real_imag(deep=deep, **hints)
        return re_part + im_part*S.ImaginaryUnit

    def _eval_is_real(self):
        return self.args[0].is_positive

    def _eval_is_bounded(self):
        arg = self.args[0]
        if arg.is_infinitesimal:
            return False
        return arg.is_bounded

    def _eval_is_positive(self):
        arg = self.args[0]
        if arg.is_positive:
            if arg.is_unbounded: return True
            if arg.is_infinitesimal: return False
            if arg.is_Number:
                return arg>1

    def _eval_is_zero(self):
        # XXX This is not quite useless. Try evaluating log(0.5).is_negative
        #     without it. There's probably a nicer way though.
        if self.args[0] is S.One:
            return True
        elif self.args[0].is_number:
            return self.args[0].expand() is S.One
        elif self.args[0].is_negative:
            return False

    def as_numer_denom(self):
        n, d = self.args[0].as_numer_denom()
        if d is S.One:
            return self.func(n), d
        return (self.func(n) - self.func(d)).as_numer_denom()

    def _eval_nseries(self, x, n):
        # NOTE Please see the comment at the beginning of this file, labelled
        #      IMPORTANT.
        from sympy import powsimp, ceiling
        if self.args[0] == x:
            return self
        arg = self.args[0]
        k, l = Wild("k"), Wild("l")
        r = arg.match(k*x**l)
        if r is not None:
            #k = r.get(r, S.One)
            #l = r.get(l, S.Zero)
            k, l = r[k], r[l]
            if l != 0 and not l.has(x) and not k.has(x):
                r = log(k) + l*log(x) # XXX true regardless of assumptions?
                return r
        order = C.Order(x**n, x)
        arg = self.args[0]
        logx = Dummy('l')
        use_lt = not C.Order(1, x).contains(arg)
        if not use_lt:
            arg0 = arg.limit(x, 0)
            use_lt = (arg0 is S.Zero)
        if use_lt: # singularity, #example: self = log(sin(x))
            # arg = (arg / lt) * lt
            lt = arg.compute_leading_term(x) # arg = sin(x); lt = x
            a = powsimp((arg/lt).expand(), deep=True, combine='exp') # a = sin(x)/x
            # the idea is to recursively call log(a).series(), but one needs to
            # make sure that log(sin(x)/x) doesn't get "simplified" to
            # -log(x)+log(sin(x)) and an infinite recursion occurs, see also the
            # issue 252.
            lt12 = lt.collect(x).as_coeff_mul(x)
            assert not lt12[0].has(x)
            obj = log(lt12[0]) + log(Mul(*lt12[1])) + log(a).nseries(x, n=n)
        else:
            # arg -> arg0 + (arg - arg0) -> arg0 * (1 + (arg/arg0 - 1))
            from sympy import cancel
            z = cancel(arg/arg0 - 1)
            if not z.has(x):
                return log((z+1)*arg0)
            lt = z.compute_leading_term(x, skip_abs=True, skip_log=True)
            lt = lt.subs(log(x), logx)
            if not lt.has(x):
                s = self.args[0].nseries(x, n=1)
                if s.subs(log(x), logx).has(x):
                    raise ValueError('expanding %s should not yield %s' % (self, s))
                # 5 is arbitrary here
                assert self.args[0].nseries(x, n=5) == s
                return log(s)
            ne = None
            if lt.is_Order:
                ne = lt.getn()
            else:
                ne = lt.leadterm(x)[1]
            num = ceiling((n+1) / ne)
            # This code is broken (it creates complicated limits), but
            # it is probably better optimised, so leave it here for reference.
            """
            print ne
            o = C.Order(z, x)
            if o is S.Zero:
                return log(1 + z) + log(arg0)
            if o.expr.is_number:
                e = log(order.expr*x)/log(x)
            else:
                e = log(order.expr)/log(o.expr)
            n = e.limit(x, 0) + 1
            if n.is_unbounded:
                # requested accuracy gives infinite series,
                # order is probably nonpolynomial e.g. O(exp(-1/x), x).
                return log(1 + z) + log(arg0)
            # XXX was int or floor intended? int used to behave like floor
            try:
                n = int(n)
            except TypeError:
                #well, the n is something more complicated (like 1+log(2))
                n = int(n.evalf()) + 1 # XXX why is 1 being added?
            assert n>=0, `n`
            """
            l = []
            g = None
            for i in xrange(num + 2):
                g = log.taylor_term(i, z, g).subs(log(x), logx)
                g = g.nseries(x, n=n)
                l.append(g)
            obj = Add(*l) + log(arg0)
        obj2 = expand_log(powsimp(obj, deep=True, combine='exp'))
        if obj2 != obj:
            r = obj2.nseries(x, n=n)
        else:
            r = obj
        r = r.subs(logx, log(x))
        if r == self:
            return self
        return r + order


    def _eval_as_leading_term(self, x):
        arg = self.args[0].as_leading_term(x)
        if arg is S.One:
            return (self.args[0] - 1).as_leading_term(x)
        return self.func(arg)

    def _sage_(self):
        import sage.all as sage
        return sage.log(self.args[0]._sage_())

class LambertW(Function):
    """Lambert W function, defined as the inverse function of
    x*exp(x). This function represents the principal branch
    of this inverse function, which like the natural logarithm
    is multivalued.

    For more information, see:
    http://en.wikipedia.org/wiki/Lambert_W_function
    """
    nargs = 1

    @classmethod
    def eval(cls, x):
        if x == S.Zero: return S.Zero
        if x == S.Exp1: return S.One
        if x == -1/S.Exp1: return S.NegativeOne
        if x == -log(2)/2: return -log(2)
        if x == S.Infinity: return S.Infinity

    def fdiff(self, argindex=1):
        if argindex == 1:
            x = self.args[0]
            return LambertW(x)/(x*(1+LambertW(x)))
        else:
            raise ArgumentIndexError(self, argindex)

