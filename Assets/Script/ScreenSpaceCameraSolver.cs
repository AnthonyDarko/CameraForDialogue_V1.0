using System;


#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;
using System;
using System.Threading;

namespace Samuel.Tools
{
    [System.Serializable, ExecuteInEditMode]
    public partial class ScreenSpaceCameraSolver : MonoBehaviour
    {
        [System.Serializable]
        public class ViewportTarget
        {
            public Transform target;
            [Range(0, 1)]
            public float compositionX = 0.33f;
            [Range(-0.5f, 0.5f)]
            public float compositionY = 0;
        }

        public enum CamClass
        {
            Type_1,
            Type_2,
            Type_3
        }

        public enum Direction
        {
            right = 1,
            left = -1
        }

        [SerializeField] private Camera _camera;
        [Space(20)]
        public ViewportTarget front;
        [Space(20)]
        public ViewportTarget back;
        [Range(-90, 90)]
        [Space(20)]
        public float yaw = -30;
        [Range(0.1f, 75)]
        [Space(20)]
        public float fov = 30;
        [Range(-90, 90)]
        [Space(20)]
        public float dutch = 0;

        public CamClass CamType;
        public Direction Dir;

        private float x_Test;// = _camera.transform.up.y;
        private float y_Test;
        private float z_Test;

        /*[SerializeField] */private float x_Right;// = _camera.transform.up.y;
        /*[SerializeField] */private float y_Right;
        /*[SerializeField] */private float z_Right;

        private float Var1;// = _camera.transform.up.y;
        private float Var2;
        private float Var3;
        private float Var4;

        public double bCompX => back.compositionX;
        public double bCompY => back.compositionY;
        public double fCompX => front.compositionX;
        public double fCompY => front.compositionY;

        public Vector3 wbPosition => back.target.position;
        public Vector3 wfPosition => front.target.position;
        private Transform _calcTarget => _camera.transform;

        private double _aspect;//屏幕长宽比
        private double _overSacle;//wb所在相机深度和wf所在相机深度的投影面大小比
        private Vector3 _lookCenter;
        private Vector3 _bp;//wb在相机水平面的投影点
        private Vector3 _fp;//wf在相机水平面的投影点
        private double _sinC;//偏航角的sin值
        private double _cosC;//偏航角的cos值
        private double _tanHalfHorizonFov;//横向fov的一半的tan值
        private double _tanHalfVerticalFov;
        private Vector3 cameraRight;
        private Vector3 camUp;
        private Vector3 camFwd;
        
        private double fl;
        private double blPfb;
        private double fb;

        public float CWf;
        [SerializeField] private float CWfDis;
        private double tanFCA;
        private double tanBCA;
        private float FCA;
        private float BCA;

        //DebugVar
        private float YawCal;
        private float temp;
        private float actualValue;

        private void Update()
        {
            Calculate();
        }

        public Transform Calculate()
        {
            if (!_calcTarget) return null;
            if (!Valid()) return null;
            CalculateCameraPos();
            return _calcTarget;
        }

        private void CalculateCameraPos()
        {
            if (bCompX == 0 || fCompX == 0) return;
            if (bCompX + fCompX == 1) return;

            switch(CamType)
            {
                case CamClass.Type_1:
                    {
                        #region Constant
                        var cAngle = Mathf.Abs(yaw);
                        _sinC = Mathf.Sin(Mathf.Deg2Rad * cAngle);
                        _cosC = Mathf.Cos(Mathf.Deg2Rad * cAngle);
                        _aspect = _camera.aspect;
                        var tanHalfVerticalFov = Mathf.Tan(Mathf.Deg2Rad * fov / 2);
                        _tanHalfHorizonFov = tanHalfVerticalFov * _aspect;
                        #endregion

                        //bCompX = 1f - 2 * bCompX;
                        #region Solve
                        //c-相机，wb-背景目标，wf-前景目标，l相机中心线和fb的交点
                        //f-前景目标在相机水平面投影，b-背景目标在相机水平面投影
                        var clPbl = _sinC / _tanHalfHorizonFov / bCompX - _cosC; // CL / BL，这里的CompX直接采用坐标位置，设中间位置为原点
                        var clPfl = _sinC / _tanHalfHorizonFov / fCompX + _cosC; // CL / FL
                        blPfb = clPfl / (clPfl + clPbl); // BL / FB
                        var flPfb = clPbl / (clPfl + clPbl); // FL / FB 
                        
                        // 下面这个式子直接写成 flPfb / blPfb 应该也是可以的，不能这么处理，这是空间中的几何，移动情况不同，裁剪空间中的点在进行投影时，压缩前和压缩后的比值可能不同
                        _overSacle = (flPfb / fCompX) / (blPfb / bCompX); // AA' / DD' :AA'为F点上相机平面的宽度，同理DD'为B点上的，这个值也可以通过aspect转为两个深度上的高度比  //写成(FL / fCompX) / (BL / bCompX) 或(FA / fCompX) / (BD / bCompX)  //后续思路可能错误！！几何意义为L点到裁剪边界的线段长度的比值(即两个平面的缩放比) 或 (FL / BL) * (bCompX / fCompX) 注意：FL / BL 和 屏幕坐标的比值可能会非常不同，取决于偏航角Yaw
                        //对两个面的比值进行重新计算，得到如下算式，假设两个平面与中轴线的交点为F'与B'，直接对CF'和CB'进行化简，得到一下式子
                        //_overSacle = (1 - _cosC / clPfl) / (1 + _cosC / clPbl); //两个面线段的比值关系，由CompX决定

                        var clPfb = clPbl * clPfl / (clPfl + clPbl); // CL / FB
                        var clPfb2 = clPfb * clPfb; // CL / FB 的平方
                        
                        //CL的值求解
                        var x = _sinC / fCompX / _aspect * 2; //这里的常量2与横坐标的范围相关，这里的X为0到1，只控制了一半的屏幕的值 // AE / FL //tan(FLD)   //_sinC / fCompX * 2 / _aspect; // (这里的常量2可以考虑去掉，常量2不能去掉，这是为什么？) 可以约为 [ sinC / (fCompX / 2 * aspect) ] => 2 * tan(DLF)，设中轴线交点为A，下边界交点为D，最后可以得到 2 * FD / FL
                        var clPdh2 = clPfl * clPfl / (abs(fCompY - bCompY / _overSacle) * x * abs(fCompY - bCompY / _overSacle) * x); // 这里的对应关系错了，需要重新解算 // bCompY * _overSacle，这是拿到bCompY在fCompY平面上投影的高度，dh的意思是fCompY与bCompY在F的投影面上的高度差
                        var clPwfwb2 = clPfb2 * clPdh2 / (clPfb2 + clPdh2);
                        var wfwb = Vector3.Distance(wbPosition, wfPosition);
                        var cl = sqrt(wfwb * wfwb * clPwfwb2); //这里采用了近似值求解，寻找更精准的方法代替，重新计算clPdh2之后正常

                        fb = cl / clPfb;
                        fl = cl / clPfl;
                        var bl = cl / clPbl; //bl的值应该是变化的，同理cl的值也应该是变化的，且比较明显
                        var bS = bl * _sinC / bCompX / _aspect * 2; // B点到下边界垂线 * 2
                        var fS = fl * _sinC / fCompX / _aspect * 2; // F点到下边界垂线 * 2
                        var fY = fS * fCompY;
                        var bY = bS * bCompY;
                        #endregion

                        //不采用迭代的方式，直接进行计算，这个值是否应该由外界决定，由设计者决定上方向的方向，这里设计者应该给出一个场景的上方向（废弃）
                        #region CalUp
                        //利用三射线定理，由相机与物体B连线，可以得到CWB与WBWF的夹角，由bY与CWB的数值关系，可以拿到他们的夹角，再由cos(WBWFF)可以拿到FWBB的余弦值，由三射线定理拿到摄像机上方向与面C-WB-WF的夹角
                        //利用二面角，做三次旋转
                        //上述想法废弃，改用Pitch，Yaw对WBWF向量的变化情况进行描述，在变化过程中，WBWF的旋转情况如何，转轴是什么向量？

                        //【解决思路】
                        //这里有一个基本的观察，因为相机没有roll角度，只围绕pitch和yaw发生了旋转，所以相机的竖屏面一定是与XZ形成的平面是垂直的
                        //先由L点做FWF的平行线，交WBWF于点N，得到一个相似三角形，由BL/FB的比值与向量WBWF相乘得到向量WBN
                        //从N点向X-Z水平面做垂线得到垂足即投影点K，此时可以得到B'K向量，由于WBB'与B'LN面垂直，所以B'K向量同时与NK向量和WBB'向量垂直
                        //此时WBK向量可以由向量WBN得到，即直接去掉y轴坐标，由此，可以得到WBK向量，WBB'向量，B'K向量，三者在长度上符合勾股定理
                        //至此，拿到cos(K_WB_B')的值，即世界坐标系下，相机偏航角的补角的余弦值，让WBK向量绕Y轴旋转该角度拿到相机的右方向向量
                        //此时，再由WBB'向量与WBN拿到B'N向量，再通过B'N，B'L，LN三者的数量关系拿到角LB'N，使B'N绕相机右方向旋转角度LB'N拿到向量B'L
                        //将B'L与向量WBB'叉乘拿到相机的上方向向量
                        var VecWbWf = wfPosition - wbPosition;
                        var VecWBN = new Vector3(VecWbWf.x * (float)blPfb, VecWbWf.y * (float)blPfb, VecWbWf.z * (float)blPfb);
                        var VecWBK = new Vector3(VecWBN.x, 0, VecWBN.z);
                        var BdotL = bl * abs(_cosC); //平移之后得到的L点并不在BdotNK平面上，所以才有平移后的L点离原L点越远误差越大的情况

                        var WBBdot = bl * abs(_sinC); //bl的值应该是在变化的，并且变化幅度较大，不应该一直保持不变
                        var BdotN2 = VecWBN.sqrMagnitude - WBBdot * WBBdot; // (LN * LN + bl * _cosC * bl * _cosC);//// BdotL * BdotL + LN * LN; //这里的计算方式需要改变，
                        var BdotK = sqrt(BdotN2 - VecWBN.y * VecWBN.y);
                        var SinBdotWBK = BdotK / (VecWBK.magnitude);
                        var BdotWBK = Mathf.Asin((float)SinBdotWBK) * Mathf.Rad2Deg;

                        //判断VecWBBdot与VecWBK的位置关系
                        {
                            if (yaw < 0)
                            {
                                cameraRight = (Quaternion.AngleAxis(-BdotWBK, Vector3.up) * VecWBK).normalized; //这个向量用于后续计算向上向量，需要注意一下方向问题，或者改变偏航角的计算方式
                            }
                            else if (yaw > 0)
                            {
                                cameraRight = (Quaternion.AngleAxis(BdotWBK, Vector3.up) * -VecWBK).normalized;
                            }
                        }

                        //Debug Info
                        var VecWfWbK = new Vector3(VecWbWf.x, 0, VecWbWf.z);
                        Var1 = (float)fb;
                        Var2 = (float)wfwb;
                        Var3 = (float)VecWbWf.y;
                        Var4 = (float)(abs(fCompY - bCompY / _overSacle) * x * fl);

                        //判断VecWBBdot与CameraRight的方向关系
                        Vector3 VecWBBdot;
                        {
                            if (yaw > 0)
                            {
                                VecWBBdot = (-1.0f) * cameraRight.normalized * (float)WBBdot;
                            }
                            else
                            {
                                VecWBBdot = cameraRight.normalized * (float)WBBdot;
                            }
                        }

                        var VecBdotK = (-1.0f) * (VecWBBdot) + VecWBK;
                        //var KBdotN = Mathf.Acos((float)(BdotK / sqrt(BdotN2))) * Mathf.Rad2Deg; //KBdotN的角度不是实际的转角，还需要加上LBdotN，拿到角KBdotL
                        var LBdotN = Mathf.Acos((float)(BdotL / sqrt(BdotN2))) * Mathf.Rad2Deg;
                        var VecBdotN = VecWBN - VecWBBdot;
                        //var LBdotK = abs((float)KBdotN) + abs((float)LBdotN); //不需要这个角度，直接旋转VecBdotN LBdotN度即可
                        //判断VecBdotN与VecLBdot（即camFwd）的位置关系
                        {
                            if (fY < bY)
                            {
                                camFwd = Quaternion.AngleAxis((float)LBdotN * (1.0f), (1.0f * cameraRight)) * -VecBdotN.normalized;
                            }
                            else
                            {
                                camFwd = Quaternion.AngleAxis((float)LBdotN * (-1.0f), (1.0f * cameraRight)) * -VecBdotN.normalized;
                            }
                        }

                        camUp = Vector3.Cross(camFwd, cameraRight);

                        //迭代部分，使用计算得到的上方向替代迭代值
                        var upAxis = camUp;
                        x_Test = _camera.transform.up.normalized.x;
                        y_Test = _camera.transform.up.normalized.y;
                        z_Test = _camera.transform.up.normalized.z;
                        _bp = wbPosition - upAxis * (float)(bY);
                        _fp = wfPosition - upAxis * (float)(fY);
                        _lookCenter = _bp * (float)flPfb + _fp * (float)blPfb;
                        var cameraFwd = Quaternion.AngleAxis(-yaw, upAxis) * (_bp - _fp).normalized;

                        x_Right = camUp.x;
                        y_Right = camUp.y;
                        z_Right = camUp.z;
                        #endregion

                        #region Apply
                        _calcTarget.transform.position = _lookCenter -
                            (float)(cl) * cameraFwd;
                        _calcTarget.transform.LookAt(_lookCenter, Quaternion.Euler(0, 0, dutch) * Vector3.up);
                        _camera.fieldOfView = fov;
                        #endregion
                    }
                    break;
                case CamClass.Type_2:
                    {
                        #region Constant
                        float cAngle;
                        _aspect = _camera.aspect;
                        _tanHalfVerticalFov = Mathf.Tan(Mathf.Deg2Rad * fov / 2);
                        _tanHalfHorizonFov = _tanHalfVerticalFov * _aspect;
                        #endregion

                        #region RecalculateYaw
                        //根据CF值重新计算Yaw值
                        CWfDis = (_camera.transform.position - wfPosition).magnitude;

                        //利用fov和aspect求出tanFCA的值，再得出FCA的度数
                        tanFCA = (fCompX) * _tanHalfVerticalFov * _aspect;
                        tanBCA = (bCompX) * _tanHalfVerticalFov * _aspect;

                        FCA = Mathf.Atan((float)tanFCA) * Mathf.Rad2Deg;
                        BCA = Mathf.Atan((float)tanBCA) * Mathf.Rad2Deg;

                        //这里先由相机与近平面的距离和tanFCA（即tanFCFdot，改为A）可以拿到近平面上的Wf在相机水平面上的投影与相机的连线
                        //而这里tanWfCF的值可以由fCompY和相机到近平面的距离得到，再通过近平面上的Wf在相机水平面上的投影与相机的连线的距离可以拿到在该平面上实际的纵坐标，再由此可以算出Wf在近平面上投影与相机的连线长度
                        //由于该条线与相机和Wf的连线是共线的，所以两者相比可以拿到比值，利用这个比值和近平面上的Wf在相机水平面上的投影与相机的连线的长度可以算出CF的值
                        //这一步有问题，重新再考虑，BWb的长度应该由CWb和bCompY与tanHalfVerticalFov来计算，同时，利用bCompY和CF还可以算出BWb与FWf之间的差值，利用这个差值就可以拿到BF的长度，后面的步骤就与水平面上的计算相同
                        //这个也有问题，考虑讲WbWf投影到AFWf平面上，此时将投影后的WbWf延BWb移动，此时得到的裁剪后FWf上的线段与投影前的线段是一致的
                        //设Wb投影变换到到AFWf平面上之后的点为Wbdot，该点的横纵坐标的具体值可以由BCompX和AF、BCompY与FWf拿到，由此可以计算得到WfWbdot的长度
                        //再由Wbdot的纵坐标和B点在该平面上的投影点与相机的连线拿到CWbdot的长度，至此，拿到CWbdot，CWf，WfWbdot三边的长度
                        //根据余弦定理，拿到∠WbdotCWf的值，该角也是三角形WfCWb的∠WfCWb，再由余弦定理，结合WbWf，CWf，∠WfCWb求出CWb的长度
                        //至此，根据三角形相似，由CWbdot与CWb的比值，可以拿到BWb的值，再由三角形相似的传递，拿到CA与CBdot（Bdot为B点在相机竖直平面上做垂线的结果）的比值
                        //至此，BBdot的实际值与ABdot的实际值全部计算完成，将（AF + BBdot） / ABdot 拿到 tan(Yaw) 的值，至此 Yaw 值计算完成
                        //这里有一个基本的观察，即在该条件下，三角锥AFWfC是一个固定的几何体

                        //由近平面转移至WfFA平面上进行计算
                        float zNear;
                        zNear = _camera.nearClipPlane;

                        float FdotAdot = zNear * (float)_tanHalfHorizonFov * (float)fCompX;
                        float CFdot = Mathf.Sqrt(FdotAdot * FdotAdot + zNear * zNear);// Mathf.Abs(zNear / Mathf.Cos(FCA));
                        float nearFWfdot = Mathf.Abs(zNear * (float)_tanHalfVerticalFov * (float)fCompY * 2);
                        float CWfdot = Mathf.Sqrt(CFdot * CFdot + nearFWfdot * nearFWfdot);
                        float CWfdotScaleCWf = Mathf.Abs(CWfdot / CWf); 
                        float CF = Mathf.Abs(CFdot / CWfdotScaleCWf);
                        float CA = Mathf.Abs(zNear / CWfdotScaleCWf);
                        float AF = CA * (float)tanFCA;
                        float FWf = nearFWfdot / CWfdotScaleCWf; //  CA * (float)_tanHalfVerticalFov * (float)fCompY; //

                        float ABa = Mathf.Abs(CA * (float)_tanHalfHorizonFov * (float)bCompX); //Mathf.Abs(CA * (float)tanBCA);
                        float BaWba = Mathf.Abs(CA * (float)_tanHalfVerticalFov * (float)bCompY * 2);
                        float CBa = Mathf.Sqrt(CA * CA + ABa * ABa);// CA / Mathf.Cos(BCA);
                        float CWba = Mathf.Sqrt(CBa * CBa + BaWba * BaWba);

                        float FBa = AF + ABa;
                        float WbaWf = Mathf.Sqrt((FWf - BaWba) * (FWf - BaWba) + FBa * FBa);

                        //余弦定理计算WbaCWf角
                        float CosWbaCWf = (CWba * CWba + CWf * CWf - WbaWf * WbaWf) / (2 * CWba * CWf);
                        float AngleWbaCWf = Mathf.Acos(CosWbaCWf) * Mathf.Rad2Deg;
                        float _fbDistance = Vector3.Distance(wbPosition, wfPosition);

                        //余弦定理求解CWb长度
                        float CWb;
                        float b = -2.0f * CWf * CosWbaCWf;
                        float a = 1f;
                        float c = (float)CWf * (float)CWf - (float)_fbDistance * (float)_fbDistance;
                        float Delta = Mathf.Sqrt(b * b - 4 * a * c);
                        if (((-b + Delta) / 2 / a) > 0) CWb = ((-b + Delta) / 2 / a);
                        else CWb = ((-b - Delta) / 2 / a);

                        //计算三角形CBBdot的所有边
                        float FB;
                        float BBdot = ABa / (CWba / CWb);
                        float BWb = BaWba / (CWba / CWb);
                        float dH = Mathf.Abs(FWf - BWb);
                        FB = Mathf.Sqrt(_fbDistance * _fbDistance - dH * dH);

                        //重新计算Yaw值
                        float SinFLA = (AF + BBdot) / FB;
                        YawCal = Mathf.Asin(SinFLA) * Mathf.Rad2Deg;
                        yaw = YawCal * (float)Dir;
                        cAngle = Mathf.Abs(yaw);
                        _sinC = Mathf.Sin(Mathf.Deg2Rad * cAngle);
                        _cosC = Mathf.Cos(Mathf.Deg2Rad * cAngle);

                        temp = YawCal;//AF;// CA;// CWfdot;// nearFWfdot;// CWfdotScaleCWf;// CA;// CWfdot / CWfdotScaleCWf;// 
                        actualValue = Vector3.Distance(_fp, wfPosition);// _camera.transform.position); // zNear;// FdotAdot;// 
                        #endregion

                        #region Solve
                        //c-相机，wb-背景目标，wf-前景目标，l相机中心线和fb的交点
                        //f-前景目标在相机水平面投影，b-背景目标在相机水平面投影
                        var clPbl = _sinC / _tanHalfHorizonFov / bCompX - _cosC; // CL / BL，这里的CompX直接采用坐标位置，设中间位置为原点
                        var clPfl = _sinC / _tanHalfHorizonFov / fCompX + _cosC; // CL / FL
                        blPfb = clPfl / (clPfl + clPbl); // BL / FB
                        var flPfb = clPbl / (clPfl + clPbl); // FL / FB                                
                        // 下面这个式子直接写成 flPfb / blPfb 应该也是可以的，不能这么处理，这是空间中的几何，移动情况不同，裁剪空间中的点在进行投影时，压缩前和压缩后的比值可能不同
                        _overSacle = (flPfb / fCompX) / (blPfb / bCompX); // AA' / DD' :AA'为F点上相机平面的宽度，同理DD'为B点上的，这个值也可以通过aspect转为两个深度上的高度比  //写成(FL / fCompX) / (BL / bCompX) 或(FA / fCompX) / (BD / bCompX)  //后续思路可能错误！！几何意义为L点到裁剪边界的线段长度的比值(即两个平面的缩放比) 或 (FL / BL) * (bCompX / fCompX) 注意：FL / BL 和 屏幕坐标的比值可能会非常不同，取决于偏航角Yaw
                                                                          
                        var clPfb = clPbl * clPfl / (clPfl + clPbl); // CL / FB
                        var clPfb2 = clPfb * clPfb; // CL / FB 的平方

                        //CL的值求解
                        var x = _sinC / fCompX / _aspect * 2;//这里的常量2与横坐标的范围相关，这里的X为0到1，只控制了一半的屏幕的值 // AE / FL //tan(FLD)   //_sinC / fCompX * 2 / _aspect; // (这里的常量2可以考虑去掉，常量2不能去掉，这是为什么？) 可以约为 [ sinC / (fCompX / 2 * aspect) ] => 2 * tan(DLF)，设中轴线交点为A，下边界交点为D，最后可以得到 2 * FD / FL
                        var clPdh2 = clPfl * clPfl / (abs(fCompY - bCompY / _overSacle) * x * abs(fCompY - bCompY / _overSacle) * x); // 这里的对应关系错了，需要重新解算 // bCompY * _overSacle，这是拿到bCompY在fCompY平面上投影的高度，dh的意思是fCompY与bCompY在F的投影面上的高度差
                        var clPwfwb2 = clPfb2 * clPdh2 / (clPfb2 + clPdh2);
                        var wfwb = Vector3.Distance(wbPosition, wfPosition);
                        var cl = sqrt(wfwb * wfwb * clPwfwb2); //这里采用了近似值求解，寻找更精准的方法代替，重新计算clPdh2之后正常

                        fb = cl / clPfb;
                        fl = cl / clPfl;
                        var bl = cl / clPbl; //bl的值应该是变化的，同理cl的值也应该是变化的，且比较明显
                        var bS = bl * _sinC / bCompX / _aspect * 2; // B点到下边界垂线 * 2
                        var fS = fl * _sinC / fCompX / _aspect * 2; // F点到下边界垂线 * 2
                        var fY = fS * fCompY;
                        var bY = bS * bCompY;
                        #endregion

                        #region CalUp
                        //【解决思路】
                        //这里有一个基本的观察，因为相机没有roll角度，只围绕pitch和yaw发生了旋转，所以相机的竖屏面一定是与XZ形成的平面是垂直的
                        //先由L点做FWF的平行线，交WBWF于点N，得到一个相似三角形，由BL/FB的比值与向量WBWF相乘得到向量WBN
                        //从N点向X-Z水平面做垂线得到垂足即投影点K，此时可以得到B'K向量，由于WBB'与B'LN面垂直，所以B'K向量同时与NK向量和WBB'向量垂直
                        //此时WBK向量可以由向量WBN得到，即直接去掉y轴坐标，由此，可以得到WBK向量，WBB'向量，B'K向量，三者在长度上符合勾股定理
                        //至此，拿到cos(K_WB_B')的值，即世界坐标系下，相机偏航角的补角的余弦值，让WBK向量绕Y轴旋转该角度拿到相机的右方向向量
                        //此时，再由WBB'向量与WBN拿到B'N向量，再通过B'N，B'L，LN三者的数量关系拿到角LB'N，使B'N绕相机右方向旋转角度LB'N拿到向量B'L
                        //将B'L与向量WBB'叉乘拿到相机的上方向向量
                        var VecWbWf = wfPosition - wbPosition;
                        var VecWBN = new Vector3(VecWbWf.x * (float)blPfb, VecWbWf.y * (float)blPfb, VecWbWf.z * (float)blPfb);
                        var VecWBK = new Vector3(VecWBN.x, 0, VecWBN.z);
                        var BdotL = bl * abs(_cosC);
                        var WBBdot = bl * abs(_sinC); //bl的值应该是在变化的，并且变化幅度较大，不应该一直保持不变
                        var BdotN2 = VecWBN.sqrMagnitude - WBBdot * WBBdot;
                        var BdotK = sqrt(BdotN2 - VecWBN.y * VecWBN.y);
                        var SinBdotWBK = BdotK / (VecWBK.magnitude);
                        var BdotWBK = Mathf.Asin((float)SinBdotWBK) * Mathf.Rad2Deg;

                        //判断VecWBBdot与VecWBK的位置关系
                        {
                            if (yaw < 0)
                            {
                                cameraRight = (Quaternion.AngleAxis(-BdotWBK, Vector3.up) * VecWBK).normalized; //这个向量用于后续计算向上向量，需要注意一下方向问题，或者改变偏航角的计算方式
                            }
                            else if (yaw > 0)
                            {
                                cameraRight = (Quaternion.AngleAxis(BdotWBK, Vector3.up) * -VecWBK).normalized;
                            }
                        }

                        //Debug info
                        var VecWfWbK = new Vector3(VecWbWf.x, 0, VecWbWf.z);
                        Var1 = (float)fb;
                        Var2 = (float)wfwb;
                        Var3 = (float)VecWbWf.y;
                        Var4 = (float)(abs(fCompY - bCompY / _overSacle) * x * fl);

                        //判断VecWBBdot与CameraRight的方向关系
                        Vector3 VecWBBdot;
                        {
                            if (yaw > 0)
                            {
                                VecWBBdot = (-1.0f) * cameraRight.normalized * (float)WBBdot;
                            }
                            else
                            {
                                VecWBBdot = cameraRight.normalized * (float)WBBdot;
                            }
                        }

                        var VecBdotK = (-1.0f) * (VecWBBdot) + VecWBK;
                        var LBdotN = Mathf.Acos((float)(BdotL / sqrt(BdotN2))) * Mathf.Rad2Deg;
                        var VecBdotN = VecWBN - VecWBBdot;

                        //判断VecBdotN与VecLBdot（即camFwd）的位置关系
                        {
                            if (fY < bY)
                            {
                                camFwd = Quaternion.AngleAxis((float)LBdotN * (1.0f), (1.0f * cameraRight)) * -VecBdotN.normalized;
                            }
                            else
                            {
                                camFwd = Quaternion.AngleAxis((float)LBdotN * (-1.0f), (1.0f * cameraRight)) * -VecBdotN.normalized;
                            }
                        }

                        camUp = Vector3.Cross(camFwd, cameraRight);

                        //迭代部分，使用计算得到的上方向替代迭代值
                        var upAxis = camUp;
                        x_Test = _camera.transform.up.normalized.x;
                        y_Test = _camera.transform.up.normalized.y;
                        z_Test = _camera.transform.up.normalized.z;
                        _bp = wbPosition - upAxis * (float)(bY);
                        _fp = wfPosition - upAxis * (float)(fY);
                        _lookCenter = _bp * (float)flPfb + _fp * (float)blPfb;
                        var cameraFwd = Quaternion.AngleAxis(-yaw, upAxis) * (_bp - _fp).normalized;

                        x_Right = camUp.x;
                        y_Right = camUp.y;
                        z_Right = camUp.z;
                        #endregion

                        #region Apply
                        _calcTarget.transform.position = _lookCenter -
                            (float)(cl) * cameraFwd;
                        _calcTarget.transform.LookAt(_lookCenter, Quaternion.Euler(0, 0, dutch) * Vector3.up);
                        _camera.fieldOfView = fov;
                        #endregion
                    }
                    break;
            }
        }

        private bool Valid()
        {
            bool isValid = _calcTarget && !PositionEquals(wfPosition, wbPosition);
            if (!isValid)
            {
                Debug.LogError($"Solver not valid {(_calcTarget == null)} ftPos: {wfPosition} btPos: {wbPosition}");
            }
            return isValid;
        }

        private bool PositionEquals(Vector3 a, Vector3 b)
        {
            return Vector3.Distance(a, b) < 0.01f;
        }

        double abs(double x)
        {
            return x < 0 ? -x : x;
        }

        double sqrt(double x)
        {
            return Mathf.Sqrt((float)x);
        }

    }
}
