function matchedIndex = matchValidPair(imageUsed, lidarUsed)
    matchedIndex = zeros(size(imageUsed));

    for i = 1 : length(imageUsed)
        if imageUsed(i) && lidarUsed(i)
            matchedIndex(i) = 1;
        else
            matchedIndex(i) = 0;
        end
    end

    matchedIndex = matchedIndex == 1;
end