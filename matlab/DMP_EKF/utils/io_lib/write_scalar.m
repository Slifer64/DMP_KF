%% write_mat function
%  \brief Writes a scalar value in stream 'fid'.
%  \details Writes the scalar value in the format specified by 'binary' flag. If the format is binary,
%           the class type of 'scalar' is used to determine the number of bits to use.
%  @param[in] scalar: scalar value
%  @param[in] fid: The output stream (optional, default = 1 for output to screen).
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[in] precision precision in txt format (optional, default = 6).
function write_scalar(scalar, fid, binary, precision, type)

    if (nargin < 2), fid = 1; end % write to screen
    if (nargin < 3), binary = false; end % text format
	if (nargin < 4), precision = 6; end % used in text format
	
    if (binary)
        fwrite(fid, scalar, class(scalar));
    else     
		s = num2str(scalar, precision);
        fprintf(fid, '%s', s);
    end

    
end
