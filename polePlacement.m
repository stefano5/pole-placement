function K = polePlacement(A, B, phiDes, education, desideredSystem)
    %if you do not have this pakage use: sudo apt install octave-symbolic
    %having this package is mandatory for this script
    pkg load symbolic 

    n = size(A); 
    m = size(B);

    %Check size of matrix. A must be n*n, B n*1 (ONLY SISO SYSTEM ARE ALLOWED)
    if (n(1) != n(2)) 
        disp("A must be square matrix");
        K="error :(";
        return;
    endif
    if (m(2) != 1) 
        disp("B must be n*1");
        K="error :(";
        return;
    endif

    %Check OK
    n = n(2);
    m = m(1);

    dim_phi_d = size(coeffs(expand(phiDes)))(2)-1;

    if (dim_phi_d != n)
        printf("Size of characteristic polynomial not check with A's size, it does not make sense. (A=%d), (phiDes:%d)\n", n, dim_phi_d);
        K="error :("
        return;
    endif

    %Check if system is reachable
    R=[];
    for i = 1:n
        R = [R A^(i-1)*B];
    endfor

    if (rank(R) != n) 
        disp("Is not possible use this algorithm with no reachable system");
        K="error :(";
        return;
    endif

    edu=0;
    if (exist("education", "var"))
        if (strcmp(education, "edu"))
            edu=1;
        endif
    endif

    %desidered_poles = roots(double(coeffs(phiDes)));
    if (edu == 1)
        disp("Reachability matrix:");
        R
    endif

    %Trasform system into canonical form (FCdC):
    %Build Ac
    Ac = [zeros(n, 1) eye(n, n-1)];

    Ac(n, n) = 0;
    syms _lambda;
    coeff_charact_polynom = double(coeffs(det(_lambda*eye(n) - A)));    %Compute the characteristic polynomial ed take their coefficient

    j=1;
    for i=1:size(coeff_charact_polynom)(2)-1
        Ac(n,i) = -coeff_charact_polynom(j);
        j++;
    endfor
    clear j

    %Build Bc
    Bc=zeros(m,1);
    Bc(m)=1;
    
    if (edu == 1)
        disp("System in canonical (control) form:");
        Ac
        Bc
    endif

    %Now try to found Kc

    coeff_charact_polynom_des = double(coeffs(phiDes));

    Kc=[];
    for i=1:n
        Kc=[Kc -Ac(n,i)-coeff_charact_polynom_des(i)];
    endfor

    clear _lambda;
    
    if (edu==1) 
        Kc
        check_pole_placement_standard_form_with_Kc = eig(Ac+Bc*Kc);
    endif
   
    Rc=[];
    for i = 1:n
        Rc = [Rc Ac^(i-1)*Bc];
    endfor

    K=Kc*inv(R*inv(Rc));
    if (edu==1)
        disp("The formula for converting Kc into K is: K=Kc*inv(R*inv(Rc))");
        R
        Rc
        format rat
        inv_Rc = inv(Rc)
        inv_R_multiplied_inv_Rc = inv(R*inv(Rc))
        format
        K
    endif

    %Check K, if you need do it
    if (edu==1)
        check_pole_placement=eig(A+B*K)
    endif
    
    if (exist("desideredSystem", "var"))
        if (strcmp(desideredSystem, "check"))
            disp("Sistem with controller:");
            sys = A+B*K
        elseif (strcmp(desideredSystem, "osservatore"))
            disp("Sistem with TODO :");
            H = -K';
            sys = A'-H*B'
        else
            sys=0;
        endif
        check_new_eigenvalues = eig(sys)
    elseif (exist("education", "var"))
        if (strcmp(education, "check"))
            disp("Sistem with controller:");
            sys = A+B*K
        elseif (strcmp(education, "osservatore"))
            disp("Sistem with TODO :");
            H = -K';
            sys = A'-H*B'
        else
            sys=0;
        endif
        check_new_eigenvalues = eig(sys)
    endif
endfunction

function [O r] = observability(A, C)

    n = size(A); 
    m = size(C);

    %Check size of matrix. A must be n*n, B n*1 
    if (n(1) != n(2)) 
        disp("A must be square matrix");
        [O r]=["error :(" 0];
        return;
    endif

    if (m(1) != 1) 
        disp("C must be 1*n");
        [O r]=["error :(" 0];
        return;
    endif

    %Check OK
    n = n(2);
    m = m(1);

    O=[];
    for i = 1:n
         O = [O ; C*A^(i-1)];
    endfor

    rho=rank(O);

    r = rho;
    if (rho != n) 
        printf("The system is NOT completely observable, has rank: %d. Size:%d \n", rho, n);
    else 
        disp ("The system is completely observable, has full rank");
    endif
endfunction


function [R r] = reachability(A, B)

    n = size(A); 
    m = size(B);

    %Check size of matrix. A must be n*n, B n*1 
    if (n(1) != n(2)) 
        disp("A must be square matrix");
        [R r]= [-1 -1];
        return;
    endif

    if (m(2) != 1) 
        disp("B must be n*1");
        [R r]= [-1 -1];
        return;
    endif

    %Check OK
    n = n(2);
    m = m(1);

    R=[];
    for i = 1:n
        R = [R A^(i-1)*B];
    endfor

    rho=rank(R);

    if (rho != n) 
        printf("The system is NOT completely reachable, has rank: %d. Size:%d \n", rho, n);
    else 
        disp ("The system is completely reachable, has full rank");
    endif

    r = rho;
endfunction
