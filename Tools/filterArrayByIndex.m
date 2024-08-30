function filteredArray = filterArrayByIndex(structArray, index)
    % Effectiveness checking
    if ~isstruct(structArray) || numel(structArray) == 0
        error('The input must be a non-empty struct array.');
    end

    if ~isvector(index) || ~isnumeric(index) || any(index < 1) || any(index > numel(structArray))
        error('The input index must be a positive integer vector and not exceed the scope of the struct array. ');
    end

    filteredArray = structArray(index);
end