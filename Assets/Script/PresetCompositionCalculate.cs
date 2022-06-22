using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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

    // Start is called before the first frame update
    void Start()
    {
        CalPresetParameter();
    }

    private void CalPresetParameter()
    {
        //��������Ŀ��ľ���ֱ��ͨ�����������������õ�
        //ֱ��������������Ŀ������������ƽ��ķ�����������ڸ���ƽ���ϵ�ͶӰ��Ȼ��������fov������ƽ����ʵ�ʵĺ������߳��ȣ��ٽ���õ�X Y��ֵ
        //����������жϿ������������Ŀ����ˮƽ���ϵ�ͶӰ�������ǰ������������ж�
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
