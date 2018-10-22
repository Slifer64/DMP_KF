%% read_rowVec function
%  \brief Reads a row vector from stream 'fid'.
%  \details Reads the number of columns and then the elements of the vector.
%  @param[in] fid: The input stream.
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[in] type: The class type of 'v' (optional, default = 'double').
%  @param[out] v: The row vector
function v = read_rowVec(fid, binary, type)

    if (nargin < 2), binary = false; end % text format
    if (nargin < 3), type = 'double'; end

	n_cols = read_scalar(fid, binary, 'int64');
    
    v = read_mat2(fid, 1, n_cols, binary, type);

end
