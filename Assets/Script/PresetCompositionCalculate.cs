using System;

#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;
using System.Threading;
using System.Collections;
using System.Collections.Generic;

[System.Serializable, ExecuteInEditMode]
public class PresetCompositionCalculate : MonoBehaviour
{
    [Space(20)]
    public Transform front;
    [Space(20)]
    public Transform back;
    [Space(20)]
    [SerializeField] private Camera _camera;

    [SerializeField] private float frontCompositionX;
    [SerializeField] private float frontCompositionY;
    [SerializeField] private float backCompositionX;
    [SerializeField] private float backCompositionY;

    [SerializeField] private float CWfDistance;
    /*[SerializeField] */private Vector3 CWfVector;
    /*[SerializeField] */private float CWbDistance;
    /*[SerializeField] */private Vector3 CWbVector;

    private float _tanHalfHorizonFov;//����fov��һ���tanֵ
    private float _tanHalfVerticalFov;

    [SerializeField] private Vector3 Test1;
    [SerializeField] private Vector3 Test2;
    [SerializeField] private float Dir;

    // Start is called before the first frame update
    void Start()
    {
        Dir = -1;
    }

    private void CalPresetParameter()
    {
        #region CompositionCalculate
        //��������Ŀ��ľ���ֱ��ͨ�����������������õ�
        CWfVector = front.transform.position - _camera.transform.position;
        CWbVector = back.transform.position - _camera.transform.position;
        CWfDistance = CWfVector.magnitude;
        CWbDistance = CWbVector.magnitude;
        //ֱ��������������Ŀ������������ƽ��ķ�����������ڸ���ƽ���ϵ�ͶӰ��Ȼ��������CA�ĳ��Ⱥ�fov������ƽ����ʵ�ʵĺ������߳��ȣ��ٽ���õ�X Y��ֵ
        Vector3 camRight = _camera.transform.right.normalized;
        Vector3 camUp = _camera.transform.up.normalized;
        Vector3 camFwd = _camera.transform.forward.normalized;

        Vector3 VecCF = CWfVector - (camUp) * Vector3.Dot(CWfVector, camUp);
        Vector3 VecCB = CWbVector - (camUp) * Vector3.Dot(CWbVector, camUp);
        Vector3 VecCA = camFwd * Vector3.Dot(CWfVector, camFwd);
        Vector3 VecCBdot = camFwd * Vector3.Dot(CWbVector, camFwd);

        _tanHalfVerticalFov = Mathf.Tan(Mathf.Deg2Rad * _camera.fieldOfView / 2);
        _tanHalfHorizonFov = _tanHalfVerticalFov * _camera.aspect;

        float FWf = Mathf.Sqrt(CWfVector.magnitude * CWfVector.magnitude - VecCF.magnitude * VecCF.magnitude);
        float BWb = Mathf.Sqrt(CWbVector.magnitude * CWbVector.magnitude - VecCB.magnitude * VecCB.magnitude);
        float AF = Mathf.Sqrt(VecCF.magnitude * VecCF.magnitude - VecCA.magnitude * VecCA.magnitude);
        float BBdot = Mathf.Sqrt(VecCB.magnitude * VecCB.magnitude - VecCBdot.magnitude * VecCBdot.magnitude);

        frontCompositionX = AF / (VecCA.magnitude * _tanHalfHorizonFov);
        frontCompositionY = FWf / (VecCA.magnitude * _tanHalfVerticalFov) / 2;
        backCompositionX = BBdot / (VecCBdot.magnitude * _tanHalfHorizonFov);
        backCompositionY = BWb / (VecCBdot.magnitude * _tanHalfVerticalFov) / 2;
        #endregion

        #region DirectionJudge
        //ͨ��VecCWf��VecCWb������ˮƽ���Լ���ƽ���ϵ�ͶӰ���ж���������������ĵ����¹�ϵ
        Vector3 VecCWbVertical = CWbVector - (camRight) * Vector3.Dot(CWbVector, camRight);
        Vector3 VecCWfVertical = CWfVector - (camRight) * Vector3.Dot(CWfVector, camRight);
        Test1 = Vector3.Cross(camFwd, VecCWbVertical).normalized;
        Test2 = camRight.normalized;
        if(Test1 == Test2)
        {
            backCompositionY *= -1f;
        }
        Test1 = Vector3.Cross(camFwd, VecCWfVertical).normalized;
        Test2 = camRight.normalized;
        if (Test1 == Test2)
        {
            frontCompositionY *= -1f;
        }

        //�ж���������ߵ���໹���Ҳֱ࣬���ж�VecCA��VecCF�Ĺ�ϵ����
        Test1 = Vector3.Cross(camFwd, VecCF).normalized;
        Test2 = camUp.normalized;
        if (Test1 == Test2)
        {
            Dir = -1;
        }
        else Dir = 1;
        #endregion
    }

    // Update is called once per frame
    void Update()
    {
        CalPresetParameter();
    }

    void DirectionJudge()
    {

    }
}
