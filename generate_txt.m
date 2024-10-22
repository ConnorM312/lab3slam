function generate_txt(idx2num, landmarks_pos)
%generate_txt Generate the output TXT of the landmarks
%   In the format specified on Wattle.
    output = {};
    for i = 1:length(idx2num)
        if i == 1
            output{end+1} = strcat('{"', num2str(idx2num(1,i)), '": [', num2str(round(landmarks_pos(1,i),1)), ', ', num2str(round(landmarks_pos(2,i),1)), ']');
        elseif i == length(idx2num)
            output{end+1} = strcat('"', num2str(idx2num(1,i)), '": [', num2str(round(landmarks_pos(1,i),1)), ', ', num2str(round(landmarks_pos(2,i),1)), ']}');
        else
            output{end+1} = strcat(' "', num2str(idx2num(1,i)), '": [', num2str(round(landmarks_pos(1,i),1)), ', ', num2str(round(landmarks_pos(2,i),1)), ']');
        end
        
        %output{end+1} = num2str(idx2num(1,i));
       % output{end+1} = landmarks_pos(:,i);
    end
    %output{end+1} = '}';

    writecell(output, 'landmarks.txt',"QuoteStrings",0)
end