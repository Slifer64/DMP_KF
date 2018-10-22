%% write_rowVec function
%  \brief Writes a row vector in stream 'fid'.
%  \details Writes the number of columns and then the elements of the vector.
%  @param[in] v: The row vector
%  @param[in] fid: The output stream (optional, default = 1 for output to screen).
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[in] precision: Precision in txt format (optional, default = 6).
function write_rowVec(v, fid, binary, precision)

    if (nargin < 2), fid = 1; end % write to screen
    if (nargin < 3), binary = false; end % text format
    if (nargin < 4), precision = 6; end % used in text format

    n_cols = int64(length(v));
    
    write_scalar(n_cols, fid, binary, precision);
    if (~binary), fprintf(fid,'\n'); end

    write_mat2(v, 1, n_cols, fid, binary, precision);
end
