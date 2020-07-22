function [angle] = normAngle(angle)
%NORMANGLE 이 함수의 요약 설명 위치
%   자세한 설명 위치
   while(abs(angle) >= 180) %각도 변환 부분
        if(angle > 0)
            angle= angle - 360;
        else
            angle = angle + 360;
        end
        
        if (abs(angle) < 180)
            break;
        end
    end
end

