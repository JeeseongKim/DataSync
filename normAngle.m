function [angle] = normAngle(angle)
%NORMANGLE �� �Լ��� ��� ���� ��ġ
%   �ڼ��� ���� ��ġ
   while(abs(angle) >= 180) %���� ��ȯ �κ�
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

