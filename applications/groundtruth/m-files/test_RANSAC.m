clear H score ok ;
numMatches = size(X1, 2)
subset = [4    13    14    22];
for t = 1:1
  % estimate homograpyh
%   subset = vl_colsubset(1:numMatches, 4)
  A = [] ;
  for i = subset
      vl_hat(X2(:,i))
      kron(X1(:,i)', vl_hat(X2(:,i)))
    A = cat(1, A, kron(X1(:,i)', vl_hat(X2(:,i)))) ;
  end
  A
  [U,S,V] = svd(A) ;  
  H{t} = reshape(V(:,9),3,3) 

  % score homography
  X2_ = H{t} * X1 ;
  du = X2_(1,:)./X2_(3,:) - X2(1,:)./X2(3,:) ;
  dv = X2_(2,:)./X2_(3,:) - X2(2,:)./X2(3,:) ;
  ok{t} = (du.*du + dv.*dv) < 6*6 ;
  score(t) = sum(ok{t}) ;
end
[score, best] = max(score) 
best
H = H{best} 
ok = ok{best}
