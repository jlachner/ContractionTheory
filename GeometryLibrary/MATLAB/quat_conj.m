function quat_new = quat_conj( quat )
% ===========================================================================
% Descriptions
% ------------
%    Quaternion Conjugation
% 
% Parameters
% ----------
%   (1) quat: 1x4 (or 4x1) quaternion vector
% 
% Returns
% -------
%   (1) quat_new: conjugated version of quat
%
% ===========================================================================

% The quaternion must either be a column or row vector
assert( isrow( quat ) || iscolumn( quat ) );
assert( length( quat ) == 4 );

% Create a new quaternion
quat_new = zeros( size( quat ), 'like', quat );

% Return the conjugate version of quaternion
quat_new( 1   ) =  quat( 1 );
quat_new( 2:4 ) = -quat( 2:4 );

end

