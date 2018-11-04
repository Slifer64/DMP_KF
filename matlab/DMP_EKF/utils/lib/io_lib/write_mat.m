%% write_mat function
%  \brief Writes a 2D matrix in stream 'fid'.
%  \details Writes first the number of rows and columns and then the elements of the matrix row by row.
%  @param[in] m: The 2D matrix
%  @param[in] fid: The output stream (optional, default = 1 for output to screen).
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[in] precision: Precision in txt format (optional, default = 6).
function write_mat(m, fid, binary, precision)

    if (nargin < 2), fid = 1; end % write to screen
    if (nargin < 3), binary = false; end % text format
    if (nargin < 4), precision = 6; end % used in text format
    
    n_rows = int64(size(m,1));
    n_cols = int64(size(m,2));
    
    write_scalar(n_rows, fid, binary, precision);
    if (~binary), fprintf(fid, '\n'); end
    write_scalar(n_cols, fid, binary, precision);
    if (~binary), fprintf(fid, '\n'); end
    
    write_mat2(m, n_rows, n_cols, fid, binary, precision);
    
end
