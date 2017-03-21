% notch_sym.m

syms b0 b1 b2 a1 a2 r theta w x  real
syms z

b0 = 1;
b1 = -2*cos(theta);
b2 = 1;

a1 = -2*r*cos(theta);
a2 = r^2;

Hz = (b0 + b1*z^(-1) + b2*z^(-2))/(1 + a1*z^(-1) + a2*z^(-2))
Hw = subs(Hz,z,exp(i*w))
Hx = subs(Hz,z,1+i*x)



» Mx0 = subs(Mx,theta,0)
 
Mx0 =
 
(x^4/(1-4*r+2*x^2+6*r^2-4*x^2*r+x^4+2*x^2*r^2+r^4-4*r^3))^(1/2)
 
» pretty(Mx0,200)
 
                                                                /                                  4                                \1/2
                                                                |                                x~                                 |
                                                                |-------------------------------------------------------------------|
                                                                |               2       2       2        4       2   2     4       3|
                                                                \1 - 4 r~ + 2 x~  + 6 r~  - 4 x~  r~ + x~  + 2 x~  r~  + r~  - 4 r~ /
» pretty(Mx0,100)
 
              /                                  4                                \1/2
              |                                x~                                 |
              |-------------------------------------------------------------------|
              |               2       2       2        4       2   2     4       3|
              \1 - 4 r~ + 2 x~  + 6 r~  - 4 x~  r~ + x~  + 2 x~  r~  + r~  - 4 r~ /
» temp = simple(Mx0)
 
temp =
 
x^2/(x^2+1-2*r+r^2)
 
» Mx0 = simple(Mx0)
 
Mx0 =
 
x^2/(x^2+1-2*r+r^2)
 
» pretty(Mx0,100)
 
                                                  2
                                                x~
                                        --------------------
                                          2                2
                                        x~  + 1 - 2 r~ + r~
» solve(Mx0,r)
??? Error using ==> solve
Error, (in fsolve) x, is in the equation, and is not solved for

Error in ==> C:\MATLAB\toolbox\symbolic\@sym\solve.m
On line 49  ==> [varargout{1:max(1,nargout)}] = solve(S{:});

» solve(Mx0-0.5,r)
 
ans =
 
[ -x+1]
[  x+1]






M = sqrt(Hw*conj(Hw));
M = simplify(M)
pretty(M,150)

Mx = sqrt(Hx*conj(Hx));
Mx = simplify(Mx)
pretty(Mx,150)

theta = pi/4
r = .9
% Find the maximum value of the transfer function (it is at an end point):
f1 = subs(M,w,pi)
f2 = subs(M,w,0)
if theta <= pi/2
  maxMval = eval(f1)
else
  maxMval = eval(f2)
end

syms maxM  real
M = M/maxM      % normalize maximum transfer function to one

figure, ezplot(eval(subs(M,maxM,maxMval)),[0,pi]), zoom on

% Find the value of w where the magnitude transfer function is equal to 0.5:
desired = 0.5;
s = solve(M-desired,w)
pretty(s,300)

% Plot solution points:
hold on
plot([0 pi],[desired desired],'r-.');
plot([eval(subs(s(1),maxM,maxMval)) eval(subs(s(2),maxM,maxMval))], [desired desired], 'go')

% compute the width of the notch
width = s(1) - s(2)
% width = simplify(width)
pretty(width,300)

ccode(width)

syms r    real      % r to symbolic form

figure, ezplot(eval(subs(width,maxM,maxMval)),[.5 .99])
