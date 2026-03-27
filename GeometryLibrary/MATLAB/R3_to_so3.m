function so3 = R3_to_so3( vec )
% ===========================================================================
% Descriptions
% ------------
%    Change from R3 vector to so3 skew-symmetric form
% 
% Parameters
% ----------
%   (1) vec: 3x1 (or 1x3) vector
% 
% Returns
% -------
%   (1) so3: the skew-symmetric form of vec
%
% ===========================================================================

% Should either be a column or row vector
assert( iscolumn( vec ) || isrow( vec ) );

% The length should be three
assert( length( vec ) == 3 );

so3 = [         0, -vec( 3 ),  vec( 2 ); ...
         vec( 3 ),         0, -vec( 1 ); ... 
        -vec( 2 ),  vec( 1 ),        0 ];
         
end