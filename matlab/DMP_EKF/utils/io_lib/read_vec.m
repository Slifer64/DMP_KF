%% read_vec function
%  \brief Reads a column vector from stream 'fid'.
%  \details Reads the number of rows and then the elements of the vector.
%  @param[in] fid: The input stream.
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[in] type: The class type of 'v' (optional, default = 'double').
%  @param[out] v: The column vector
function v = read_vec(fid, binary, type)

    if (nargin < 2), binary = false; end % text format
    if (nargin < 3), type = 'double'; end
    
    n_rows = read_scalar(fid, binary, 'int64');
    
    v = read_mat2(fid, n_rows, 1, binary, type);
    
end
