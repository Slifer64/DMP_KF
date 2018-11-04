%% write_mat function
%  \brief Reads a scalar value from stream \a in.
%  \details Reads a scalar value in the format specified by \a binary flag.
%  @param[out] scalar scalar value.
%  @param[in] fid: The input stream.
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[in] type: The class type of 'scalar' (optional, default = 'double').
function scalar = read_scalar(fid, binary, type)

    if (nargin < 2), binary = false; end % text format
    if (nargin < 3), type = 'double'; end
	
    if (binary)
		scalar = fread(fid, 1, type);
    else     
		s = fscanf(fid,'%s', 1);
		scalar = str2num(s);
    end

    
end
