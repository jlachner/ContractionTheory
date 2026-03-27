function [Ctp,Theta_p,tp,dp,I] =BezierProjection(IN,P,varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function calculated the projection of point P onto the rational
% beziercurve described by either a set of control points or a struct
% containing the relevant fields.
%
% Syntax:
%[Ctp,theta,tp,dp,I] = BezierProjection(Q,p)   %for regular bezier curve
%[Ctp,theta,tp,dp,I] = BezierProjection(Q,p,w) %for rational bezier curve
%[Ctp,theta,tp,dp,I] = BezierProjection(Bez,p) %for bezier spline
%
% Inputs:
% IN can be either:
%       Q(m,n)          m dimenional space, n control points
%       Bez             a struct containing fields Q and w
% p(m,1)                column vector, single point in R^n space
% w(1,n)                row vector, weights per control point (optional)
%                       and only used if IN is of type Q
% Outputs:
% C(m,l)        projection of point on bezier curve
% Theta_p(1,l)  angle of bezier curve at location of projection
% tp            tp minimizes B(tp)-p
% dp            distance to projection (negative implies to the right of
%               bezier curve)
% I             index to the segment at which point is closest
%
% Update 12 may 2020: 
% - Included bounding box upper and lower bound as suggested by Douglas
% Ridgeway, this speeds up the code significantly as not all the segments
% need to be evaluated.
% - Corrected the code for 3D curves
%
% Eindhoven University of Technology
% Robbin van Hoek - okt 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% checks on inputs
if isstruct(IN)
    Bez=IN;
    structmode=1;
else
    Q=IN;
    structmode=0;
    if nargin==3
        w=varargin{1};
    else
        w = ones(1,size(Q,2));
    end
end
  
if structmode == 1
    upperbound = zeros(numel(Bez),1);
    lowerbound = zeros(numel(Bez),1);
    
    %bounding boxes cartesian
    for ii=1:numel(Bez)
        BBmin = min(Bez(ii).Q, [], 2);
        BBmax = max(Bez(ii).Q, [], 2);
        near_deltavec = max(0, BBmin - P) - max(0, P-BBmax);
        lowerbound(ii) = sum(near_deltavec.*near_deltavec);
        bc = 0.5*(BBmax+BBmin);
        far_deltavec = (BBmax - P).*(bc > P) + (BBmin - P).*(bc <= P);
        upperbound(ii) = sum(far_deltavec.*far_deltavec);
    end
    %loop over possible segments
    leastupperbound = Inf;
    while min(upperbound) > min(lowerbound)
        [~, nextseg] = min(lowerbound);
        [CTP, THETA, TP, DP] = BezierProjection(Bez(nextseg).Q, P);
        upperbound(nextseg) = abs(DP.*DP);
        lowerbound(nextseg) = abs(DP.*DP);
        if abs(DP.*DP) < leastupperbound
            %update most recent evaluation
            leastupperbound = abs(DP.*DP);
            Ctp = CTP; Theta_p = THETA; tp = TP; dp = DP;
        end
    end
else
    
    if all(w==w(1))
        rational=0;
    else
        rational=1;
    end
    
    %check if rational or regular bezier curve. if no info, assume regular
    p=size(Q,2)-1; %degree of bezier curve
    m=size(Q,1);   %dimension of space
    
    %% create bernstein basis functions: power basis
    Bp = bernsteinM_pb(p);
    
    %create transformation matrices such that kron(Up,Up) = T1* (t.^([0:p^2]')
    T1 = zeros((p+1)^2,p^2+1);
    for ii=1:(p+1)
        T1(((ii-1)*(p+1)+1):(((ii-1)*(p+1)+1)+p),ii:(ii+p)) = eye(p+1);
    end
    
    if rational==1
        
        %create transformation matrices such that kron(Upk,Up) = T2* (t.^([0:p^3]')
        T2 = zeros((p^2+1)*(p+1),p^2*p+1);
        for ii=1:(p^2+1)
            T2(((ii-1)*(p+1)+1):(((ii-1)*(p+1)+1)+p),  ii:(ii+p)) = eye(p+1);
        end
        
        %first derivative premultiplication
        Du  = diag((1:p),-1); %derivative operator (premultiplication) for derivatives
        
        %second derivative premultiplication
        Duk = zeros((p+1)^2);
        for ii=1:(p+1)
            col = zeros(2,1);
            row = zeros(2,1);
            if ii==1
                col(1) = ii-1 + 1;  %offset by 1 to remove column at index zero
                col(2) = p+ii-1; %offset by 1 to remove column at index zero
                row(1) = 1+(ii-1)*(p+1) + 1;
                row(2) = (p+1)+(ii-1)*(p+1);
                
                mt    = numel((row(1):row(2)));
                block1 = diag((ii:ii+(mt-1)));
                Duk((row(1):row(2)),(col(1):col(2))) = block1 ;
            else
                col(1) = ii-1;  %offset by 1 to remove column at index zero
                col(2) = p+ii-1; %offset by 1 to remove column at index zero
                row(1) = 1+(ii-1)*(p+1);
                row(2) = (p+1)+(ii-1)*(p+1);
                
                mt   = numel((row(1):row(2)));
                block2 = diag((ii-1:ii+(mt-2)));
                Duk((row(1):row(2)),(col(1):col(2))) = block2 ;
            end
        end
        
        %weights on control points
        W  = diag(w);
        Wu = ones(1,numel(w))*W*Bp;
        Vu = Q*W*Bp;
        
        %first dervivative coefficients
        Ak = kron(Wu,Vu*Du)-kron(Vu,Wu*Du); %see eq. A.3 of meeting 7
        
        % find projection
        
        % numerical minimization
        %  dot( Ak*kron(Up,Up), (p*wu - Vu)*Up)
        %  dot( Ak*T1*Up2, (p*wu - Vu)*Up)   %Up2 = t.^([0:p^2]')
        % then kronecker product per row and sum to represent the dot product
        Aktrans= Ak*T1;
        term   = (P*Wu - Vu);
        dotprod=zeros(size(term,1),size(Aktrans,2)*size(term,2));
        for kk=1:size(term,1)
            dotprod(kk,:) = kron(Aktrans(kk,:),term(kk,:));
        end
        g4   = sum(dotprod*T2,1);%*Up3;
    else
        Bpd= bernsteinM_pb(p-1);
        Bpd= [Bpd, zeros(size(Bpd,1),1)]; %power basis vector is created for order p, so add column of zeros at highest order.
        
        D = [ones(m,1), zeros(m,size(Q*Bp,2)-1)];
        Qd= Hodograph(Q);
        
        % find projection
        % find roots of    g = Bd*(P-B)
        Aktrans = Qd*Bpd;   
        term    = (P.*D - Q*Bp);
        dotprod=zeros(size(term,1),size(Aktrans,2)*size(term,2));
        for kk=1:size(term,1)
            dotprod(kk,:) = kron(Aktrans(kk,:),term(kk,:));
        end
        g4   = sum(dotprod*T1,1);%*Up3;
    end
    
    %find roots of polynomial
    tp   = roots(fliplr(g4)); %ROOTS RETURNED ARE IN SIMULINK ARE DIFFERENT THEN MATLAB!
    
    %See which roots are valid
    tol = 10*eps; %tolerance for imaginairy roots
    temp  = real((abs(imag(tp))<tol).*tp); %set imaginairy roots to 0
    temp(temp>1)=1; %set larger then 1 to 1
    temp(temp<0)=0; %set smaller then 0 to 0
    temp3 = unique(temp); %C = A(IA) and A = C(IC) (or A(:) = C(IC)
    
    %preallocate arrays to fit results for unique roots
    m=size(Q,1);   %dimension of space
    CTP = zeros(m,numel(temp3));
    DP  = zeros(1,numel(temp3));
    for ii=1:numel(temp3)   %loop over unique and valid roots
        %check position at found point
        if temp3(ii)==0
            ctp=Q(:,1);
        elseif temp3(ii)==1
            ctp=Q(:,end);
        else
            [ctp] =BezierEval(Q,temp3(ii),w);
        end
        
        CTP(:,ii)=ctp; %coordinate itself
        DP(ii)=norm(ctp-P); %distance to point (d coordinate)
    end
    
    %find minimum value per segment
    [Dpa_abs,Imin]= min(DP);
    Ctp     = CTP(:,Imin);
    tp      = temp3(Imin);
    
    if tp==1
        Cd = (w(end-1)/w(end))*(Q(:,end)-Q(:,end-1));
        Theta_p = atan2(Cd(2),Cd(1));
    elseif tp==0
        Cd = (w(2)/w(1))*(Q(:,2)-Q(:,1));
        Theta_p = atan2(Cd(2),Cd(1));
    else
        [~,Theta_p] =BezierEval(Q,tp,w); %add angle to minimum distance point
        %Cd = [cos(Theta_p);sin(Theta_p)];
    end
    
    %find if the point is to the left or the right of the line
    %vector from projection to point = P-Ctp
    %unitvector from projection in path direction = Cd
    phi  = atan2(P(2)-Ctp(2),P(1)-Ctp(1));
    dp = sign( mod(phi-mod(Theta_p,2*pi)+pi,2*pi)-pi)*Dpa_abs;

end
