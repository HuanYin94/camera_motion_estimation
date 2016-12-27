function [ finalRT ] = estimate( ref, cur, depth )
%   Detailed explanation goes here

    s0 = 8;
    s = s0;
    RTmin = eye(4);
    for n = 1 : 1 : 3

        % Threshold may be added to CANNY
        beta = 0.5;
        refImage = imresize(ref, 1/s);
        curImage = imresize(cur, 1/s);

        grayImageRef = rgb2gray(refImage);
        grayImageCur = rgb2gray(curImage);

        edgeDetectedRef = edge(grayImageRef, 'canny', 0.1);
        edgeDetectedCur = edge(grayImageCur, 'canny', 0.1);
        [curreV,curreU] = find(edgeDetectedCur>0);

        %% To prepare
        % Get DistanceTransform of Reference-Image
        distanceTransformRef = bwdist(edgeDetectedRef, 'euclidean');
        distanceTransformCur = bwdist(edgeDetectedCur, 'euclidean');

        % prepare for the second step
        Edge = distanceTransformRef == 0; % Edge = edgeDetectedRef
        edgeIndex = zeros(nnz(Edge), 2);

        % get nonzero-index of Ref
        [HRef, WRef] = size(distanceTransformRef);
        k=1;
        for i = 1 : HRef
            for j = 1 : WRef
                if Edge(i, j) == 1
                    edgeIndex(k, 1) = i;  % Y of image-cord of Ref
                    edgeIndex(k, 2) = j;  % X of image-cord of Ref
                    k = k + 1;
                else
                    %nothing
                end
            end
        end

        %
        k = k - 1;
        %

        % get Ei of left camera
        kDepth = 0.0002;
        cx = 318.6; cy = 255.3; fx = 517.3; fy = 516.5;
        cx = cx/s; cy = cy/s; fx = fx/s; fy = fy/s;
        depthOfRef = double(imresize(depth,1/s,'nearest'));

        % 3D in real space, Z = 0 filtered
        Z = zeros(k, 1); X = zeros(k, 1); Y = zeros(k, 1);
        p = 1;
        filteredIndex = zeros(k, 2);
        for i = 1 : k
            Zi = kDepth*depthOfRef(edgeIndex(i, 1), edgeIndex(i, 2)) ;
            if Zi == 0
                continue;
            else
                Z(p) = Zi;
                X(p) = Z(p) * (edgeIndex(i, 2) - cx) / fx;
                Y(p) = Z(p) * (edgeIndex(i, 1) - cy) / fy;
                filteredIndex(p, 1) = edgeIndex(i, 1);
                filteredIndex(p, 2) = edgeIndex(i, 2);
                p = p + 1;
            end
        end

        % p is the num of non-zero-depth edge points of Reference-pic
        p = p - 1;
        %
        Z = Z(1 : p); X = X(1 : p); Y = Y(1 : p); filteredIndex = filteredIndex(1 : p, 1 : 2);
        %

        %3D points finished

        %% start to calculate
                
        % initial R & T = kaci etc.
        Vmin = 0;
        RT = RTmin;

        % the first step of J, whole picture
        [gdx, gdy] = imgradientxy(distanceTransformCur);     

        % error function?
        v = zeros(9999, 1);
        S = zeros(1,6);


        for m = 1 : 9999
            %clc
            if m>10
                beta = 0.9;
            end
            Rotate = RT(1 : 3, 1 : 3);
            Translate = RT(1 : 3, 4);

            q = 1;
            H = zeros(1, 6);
            Vsum = 0;
            projectedIndex = zeros(p, 2);

            % All 2 * vi * Ji,  add p-points together
            for i =1 : p
                %
                Ei = [X(i); Y(i); Z(i)];   
                EiNewT = Ei - Translate;        

                % 3D points to current-image
                EiNewRT = Rotate' * EiNewT;
                %
                uNew = round( fx * EiNewRT(1) / EiNewRT(3) + cx )+1;
                vNew = round( fy * EiNewRT(2) / EiNewRT(3) + cy )+1;
                % filter the outside points after projection 
                if  uNew <=  WRef && uNew > 0 && vNew <= HRef && vNew > 0 && Z(i)>0.1 && Z(i) < 20
                    projectedIndex(q, 1) = vNew;
                    projectedIndex(q, 2) = uNew;
                    % calculate Vi
                    Vi = distanceTransformCur(vNew, uNew);
                    Vsum = Vsum + Vi.^2;

                    % 1 * 2
                    firstDiff = [ gdx(vNew,uNew), gdy(vNew,uNew) ];
                    % 2 * 3
                    secondDiff = [ fx / Z(i), 0, -fx * X(i) * Z(i).^(-2);
                                          0, fy / Z(i), -fy * Y(i) * Z(i).^(-2) ];
                    % 3 * 6
                    EiNewTx = [ 0, -EiNewT(3), EiNewT(2);
                                       EiNewT(3), 0, -EiNewT(1);
                                       -EiNewT(2), EiNewT(1), 0];
                    thirdDiff = Rotate' * [ -eye(3), EiNewTx];
                    % Multiply together
                    Ji = firstDiff * secondDiff * thirdDiff;
                    H = H + 2 * exp(-Vi)* Vi * Ji;
                    q = q + 1;
                end

            end

            %
            % Manifold kaci addition TO Eq-space Rt mutiplication
            alpha = -5e-4;
            magicNumber = 5;
            if n < 3
                H = alpha / (magicNumber*(n-1) + 1) / m.^(0.5) * H;
            else
                H = alpha / (10*(n-1) + 1) / m.^(0.5) * H;
            end
            H = H /q;
            S = (1-beta)*H + beta*S;
            t = S(1 : 3)'; w = S(4 : 6)';  %1*6 vector???
            wx = [ 0, -w(3), w(2); 
                      w(3), 0, -w(1);
                      -w(2), w(1), 0 ];
        %     theta = norm(w);
        %     deltaRotate = eye(3) + wx * sin(norm(w, 2) * theta) / norm(w, 2) + wx.^(2) * (1 - cos(norm(w, 2) * theta)) / norm(w, 2).^(2);
        %     deltaTranslate = (eye(3) - deltaRotate) * cross(w, t) + w * w' * t * theta; 

        %     deltaRT = [deltaRotate, deltaTranslate; 0, 0, 0, 1];  
            deltaRT = expm([wx t;0 0 0 0]);

            RT = RT * deltaRT;

            %m
            %RT
            v(m) = Vsum;
            if m == 1
                Vmin = v(m);
            else
                if v(m) < Vmin
                   Vmin = v(m);
                   RTmin = RT;
                end
            end

            imshow(grayImageCur)
            hold on;
            plot(projectedIndex(:,2), projectedIndex(:,1),'g.');
            plot(curreU, curreV,'r.')
            hold off;
            
            drawnow

            %break?
            if norm( deltaRT - eye(4), 2 ) < 1e-4 * n.^(2)
                s = s /2;
                %norm( deltaRT - eye(4), 2 )
                break;
            end

        end

    end
    
    finalRT = RTmin;
end

