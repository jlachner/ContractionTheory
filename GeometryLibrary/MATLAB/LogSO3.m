function so3 = LogSO3( R )
% ===========================================================================
% Descriptions
% ------------
%    Logarithmic Map of SO3
% 
% Parameters
% ----------
%   (1) R: An SO3 matrix
% 
% Returns
% -------
%   (1) so3: the skew-symmetric part
%
% ===========================================================================

assert( is_SO3( R ) );

% First, need to check whether 
if abs( trace( R ) + 1 ) >= eps
    
    theta = acos( 1/2*( trace( R ) - 1 ) );

    if theta <= eps
        so3 = zeros( 3, 3 );
    else
        so3 = theta/( 2 * sin( theta ) ) * ( R - R' );
    end

else
    % Check if there is a plaussible solution
    if ( R( 3, 3 )+1 >= eps )
        so3 = pi * R3_to_so3( 1/sqrt( 2*( 1 + R( 3, 3 ) ) )*[ R( 1, 3 ), R( 2, 3 ), R( 3, 3 ) + 1 ] );

    elseif ( R( 2, 2 )+1 >= eps )
        so3 = pi * R3_to_so3( 1/sqrt( 2*( 1 + R( 2, 2 ) ) )*[ R( 1, 2 ), 1 + R( 2, 2 ), R( 3, 2 ) ] );
        
    else
        so3 = pi * R3_to_so3( 1/sqrt( 2*( 1 + R( 1, 1 ) ) )*[ 1 + R( 1, 1 ), R( 2, 1 ), R( 3, 1 ) ] );
    end
end