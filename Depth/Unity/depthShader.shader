// Upgrade NOTE: replaced '_Object2World' with 'unity_ObjectToWorld'



Shader "depthShader" {
	SubShader{
		Tags{ "RenderType" = "Opaque" }

		Pass{

		CGPROGRAM
	#pragma vertex vert
	#pragma fragment frag


		struct a2v {
		float4 vertex : POSITION;
		fixed4 color : COLOR;
	};

	struct v2f {
		float4 pos : SV_POSITION;
		half dist : TEXCOORD0;
	};


	v2f vert(a2v v) {
		v2f o;
		o.pos = UnityObjectToClipPos(v.vertex);
		o.dist = distance(_WorldSpaceCameraPos, mul(unity_ObjectToWorld, v.vertex)) / 5.0;
		return o;
	}

	fixed4 frag(v2f i) : COLOR{

		float gray = 1.0 - i.dist;
		return fixed4(gray, gray, gray, 1);

	}
		ENDCG
	}
	}
		FallBack Off
}
