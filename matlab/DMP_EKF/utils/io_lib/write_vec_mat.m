%% write_vec_mat function
%  \brief Writes a vector of 2D matrices in stream 'fid'
%  \details Writes the number of 2D matrices, and thern the number of rows and cols and the elements of each 2D matrix row by row.
%  @param[out] m: cell array where each cell has a 2D matrix
%  @param[in] fid: The output stream (optional, default = 1 for output to screen).
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[in] precision precision in txt format (optional, default = 6).
function write_vec_mat(m, fid, binary, precision)

    if (nargin < 2), fid = 1; end % write to screen
    if (nargin < 3), binary = false; end % text format
    if (nargin < 4), precision = 6; end % used in text format

    n_mat = int64(length(m));
    
    write_scalar(n_mat, fid, binary, precision);
    if (~binary), fprintf(fid, '\n'); end

    for k=1:n_mat
        write_mat(m{k}, fid, binary, precision);
    end
end
