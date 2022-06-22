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
        //相机与近景目标的距离直接通过两个坐标相减计算得到
        //直接利用相机与近景目标的向量与各个平面的法向量求算出在各个平面上的投影，然后再利用fov算出这个平面上实际的横纵轴线长度，再解算得到X Y的值
        //相机的左右判断可以利用与近景目标在水平面上的投影与相机的前向量计算进行判断
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
