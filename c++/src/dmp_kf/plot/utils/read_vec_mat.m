%% read_vec_mat function
%  \brief Reads a vector of 2D matrices from stream 'fid'
%  \details Reads the number of 2D matrices, the number of rows and cols and the elements of each 2D matrix row by row.
%  @param[in] fid: The input stream.
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[in] type: The class type of 'm' (optional, default = 'double').
%  @param[out] m: cell array where each cell has a 2D matrix
function m = read_vec_mat(fid, binary, type)

    if (nargin < 2), binary = false; end % text format
    if (nargin < 3), type = 'double'; end
    
    n_mat = read_scalar(fid, binary, 'int64');
    
    m = cell(n_mat,1);

    for k=1:n_mat
        m{k} = read_mat(fid, binary, type);
    end

end
