function theta = calAngle(tanA,tanB)

theta = atan((tanB - tanA)/(1 + tanB*tanA));

end