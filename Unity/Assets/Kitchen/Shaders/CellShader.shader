Shader "Universal Render Pipeline/CellShader"
{
    Properties
    {
        [Header(Base)]
        _MainTex ("Base Map", 2D) = "white" {}
        [HDR] _MainColor("Color", Color) = (0.5,0.5,0.5,1)
        [Header(Shadow)]
        _ShadowBrightness ("Shadow Brightness", Range(0,1)) = 0.5
        _ShadowBlend ("Shadow Blend Power", Range(0,5)) = 0.05
        _ShadowScale ("Shadow Scale", Range(0,1)) = 0.5
        _ShadowColor ("Shadow Tint", Color) = (0.5,0.5,0.5,1)
        [Header(Specular)]
        _SpecularSize("Specular Size", Range(0,1)) = 0
        _SpecularFalloff("Specular Falloff", Range(0,1)) = 0
        [HDR] _SpecularColor ("Specular Color", Color) = (1,1,1,1)
    }
    SubShader
    {
        Tags
        {
            "RenderType"="Transparent" "RenderPipeline"="UniversalRenderPipeline"
        }
        LOD 100

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS_CASCADE
            #pragma multi_compile _ _SHADOWS_SOFT
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                float3 normal : NORMAL;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;

                float4 vertex : SV_POSITION;
                float3 normal : NORMAL;
                float3 shadowCoord : TEXCOORD2;
                float3 viewDirection : TEXCOORD3;
            };

            Texture2D _MainTex;
            SamplerState sampler_MainTex;
            float4 _MainTex_ST;
            float4 _MainColor;
            float4 _ShadowColor;
            half _ShadowBlend;
            half _ShadowScale;
            half _ShadowBrightness;
            half _SpecularSize;
            half _SpecularFalloff;
            float _ShadowStep;
            float4 _SpecularColor;


            float remap(float val, float from, float to, float from2, float to2)
            {
                return from2 + (to2 - from2) * ((val - from) / (to - from));
            }

            v2f vert(appdata v)
            {
                v2f o;
                VertexPositionInputs vertInputs = GetVertexPositionInputs(v.vertex.xyz);
                //This function calculates all the relative spaces of the objects vertices
                o.vertex = vertInputs.positionCS;
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                o.normal = mul(unity_ObjectToWorld, v.normal).xyz;
                o.viewDirection = _WorldSpaceCameraPos - vertInputs.positionVS;
                if (unity_OrthoParams.w != 0)
                {
                    // Ortographic View
                    float4x4 viewMat = GetWorldToViewMatrix();
                    o.viewDirection = viewMat[2].xyz;
                }
                o.shadowCoord = GetShadowCoord(vertInputs).xyz;
                return o;
            }

            half4 frag(v2f i) : SV_Target
            {
                // sample the texture
                half4 col = _MainTex.Sample(sampler_MainTex, i.uv);
                //add color
                col *= _MainColor;
                // get main light
                Light mainLight = GetMainLight(TransformWorldToShadowCoord(i.shadowCoord));


                // Calculate shadow
                float4 shadow = dot(i.normal, mainLight.direction);
                float shadowScale = remap(_ShadowScale, 0, 1, -1, 1);
                float shadowBlend = shadowScale - _ShadowBlend;
                shadow = smoothstep(shadowBlend, shadowScale, shadow);
                shadow += lerp(_ShadowBrightness, shadow, shadow);
                half4 shadowColor = _ShadowColor;
                shadow = lerp(shadowColor, shadow, shadow);
                

                // Apply main light colour
                col *= half4(mainLight.color.xyz, 1);


                // Calculate Specular
                float3 reflectionDirection = reflect(mainLight.direction, i.normal);
                float specularReflection = dot(i.viewDirection, -reflectionDirection);
                specularReflection = pow(abs(specularReflection), _SpecularFalloff);
                float specularChange = fwidth(specularReflection);
                float specularSize = remap(_SpecularSize, 0, 1, -1, 1);
                float4 specularIntensity = smoothstep(1 - specularSize, 1 - specularSize + specularChange,specularReflection);
                specularIntensity *= _SpecularColor;
                col *= shadow;
                col += specularIntensity;
                return col;
            }
            ENDHLSL
        }
    }
}