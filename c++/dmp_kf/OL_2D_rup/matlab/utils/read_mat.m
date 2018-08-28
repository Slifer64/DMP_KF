%% read_mat function
%  \brief Reads a 2D matrix from stream 'fid'.
%  \details Reads first the number of rows and columns and then the elements of the matrix row by row.
%  @param[in] fid: The input stream.
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[in] type: The class type of 'm' (optional, default = 'double').
%  @param[out] m: The 2D matrix
function m = read_mat(fid, binary, type)

    if (nargin < 2), binary = false; end % text format
    if (nargin < 3), type = 'double'; end
    
    n_rows = read_scalar(fid, binary, 'int64');
    n_cols = read_scalar(fid, binary, 'int64');

    m = read_mat2(fid, n_rows, n_cols, binary, type);

end

