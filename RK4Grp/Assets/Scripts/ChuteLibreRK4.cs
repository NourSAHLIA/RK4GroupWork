using System;
using System.Collections.Generic;
using UnityEngine;

public class ChuteLibreRK4 : MonoBehaviour
{
    public float g = 9.81f;
    public float tau = 2f;

    public LayerMask obstacleLayers = ~0;
    public float tMax = 10f;
    public float restDelay = 2f;
    [Range(0.2f, 0.7f)] public float graphWidthFraction = 0.35f; 
    [Range(0.3f, 1f)] public float panelAlpha = 0.75f;

    float startZ;
    float radius;                       // rayon réel de la sphère (collider x échelle)

    float v, z, t, wait;
    bool landed;                        // sphère hits obstacle
    bool finished;                      // tMax atteint sans obstacle
    float aBefore, vBefore;             // a et v  juste avant l'impact
    List<float> tList = new List<float>(), aList = new List<float>();
    List<float> vList = new List<float>(), zList = new List<float>();
    Material mat;
    GUIStyle style;

    void Start()
    {
        startZ = transform.position.y;

        SphereCollider sc = GetComponent<SphereCollider>();
        Vector3 s = transform.lossyScale;
        radius = sc != null
            ? sc.radius * Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.y), Mathf.Abs(s.z))
            : 0.5f;

        mat = new Material(Shader.Find("Hidden/Internal-Colored"));
        mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        mat.SetInt("_Cull", 0);
        mat.SetInt("_ZTest", (int)UnityEngine.Rendering.CompareFunction.Always);
        Restart();
    }

    void Restart()
    {
        v = 0f;
        z = startZ;
        t = 0f;
        wait = 0f;
        landed = false;
        finished = false;
        tList.Clear(); aList.Clear(); vList.Clear(); zList.Clear();
        transform.position = new Vector3(transform.position.x, z, transform.position.z);
        Record(g - v / tau, v, z);
    }

    void Record(float aVal, float vVal, float zVal)
    {
        tList.Add(t); aList.Add(aVal); vList.Add(vVal); zList.Add(zVal);
    }

    void FixedUpdate()
    {
        float h = Time.fixedDeltaTime;

        // Fin de simulation :  un obstacle/ tMax atteint
        if (landed || finished)
        {
            wait += h;
            if (landed)
            {
                t += h;
                Record(0f, 0f, z);      // selon newton's second low N = m·g, donc a = 0 et v = 0 
            }
            if (wait > restDelay) Restart();
            return;
        }

        float vPrev = v, zPrev = z;
        Vector2 y = RK4Step(Derivatives, t, new Vector2(v, z), h);

        // Un obstacle 
        float d = zPrev - y.y;
        Vector3 origin = new Vector3(transform.position.x, zPrev, transform.position.z);
        if (d > 1e-6f &&
            Physics.SphereCast(origin, radius, Vector3.down, out RaycastHit hit, d,
                               obstacleLayers, QueryTriggerInteraction.Ignore))
        {
          
            float hHit = h * hit.distance / d;
            y = RK4Step(Derivatives, t, new Vector2(vPrev, zPrev), hHit);
            t += hHit;
            v = y.x;
            z = zPrev - hit.distance;   // altitude du centre de la sphère au contact
            transform.position = new Vector3(transform.position.x, z, transform.position.z);

            aBefore = g - v / tau;
            vBefore = v;
            Record(aBefore, vBefore, z);    

            // Impact inélastique (e = 0)
            v = 0f;
            landed = true;
            Record(0f, 0f, z);
            return;
        }

        // Pas d'obstacle chute continue
        v = y.x;
        z = y.y;
        t += h;
        transform.position = new Vector3(transform.position.x, z, transform.position.z);
        Record(g - v / tau, v, z);

        if (t >= tMax) finished = true;
    }

    Vector2 Derivatives(float time, Vector2 y)
    {
        return new Vector2(g - y.x / tau, -y.x);
    }

    Vector2 RK4Step(Func<float, Vector2, Vector2> f, float time, Vector2 y, float h)
    {
        Vector2 k1 = f(time, y);
        Vector2 k2 = f(time + h / 2f, y + (h / 2f) * k1);
        Vector2 k3 = f(time + h / 2f, y + (h / 2f) * k2);
        Vector2 k4 = f(time + h, y + h * k3);
        return y + (h / 6f) * (k1 + 2f * k2 + 2f * k3 + k4);
    }

    static float NiceStep(float range, int n)
    {
        float raw = range / n;
        float mag = Mathf.Pow(10f, Mathf.Floor(Mathf.Log10(raw)));
        float x = raw / mag;
        return (x < 1.5f ? 1f : x < 3f ? 2f : x < 7f ? 5f : 10f) * mag;
    }

    static float Tick(float lo, float step, int i)
    {
        float val = lo + i * step;
        return Mathf.Abs(val) < step * 1e-3f ? 0f : val;
    }

    void OnGUI()
    {
        if (Event.current.type != EventType.Repaint || tList.Count < 2) return;

        float w = Mathf.Max(Screen.width * graphWidthFraction, 340f);
        float h = Screen.height / 3f - 15f;

        if (style == null)
            style = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold };
        style.fontSize = Mathf.Clamp(Mathf.RoundToInt(w / 38f), 11, 16);

        DrawGraph(new Rect(10, 10, w, h), aList, Color.red, "Accélération a", "m/s²", aBefore);
        DrawGraph(new Rect(10, 10 + (h + 10), w, h), vList, Color.green, "Vitesse v", "m/s", vBefore);
        DrawGraph(new Rect(10, 10 + 2 * (h + 10), w, h), zList, Color.cyan, "Altitude z", "m", float.NaN);
    }

    void DrawGraph(Rect r, List<float> data, Color col, string name, string unit, float before)
    {
        GUI.color = new Color(0, 0, 0, panelAlpha);
        GUI.DrawTexture(r, Texture2D.whiteTexture);
        GUI.color = Color.white;

        Rect p = new Rect(r.x + 60, r.y + 30, r.width - 80, r.height - 60);

        // inclut 0 et gère les valeurs négatives
        float min = 0f, max = 0.001f;
        foreach (float d in data) { min = Mathf.Min(min, d); max = Mathf.Max(max, d); }
        float ystep = NiceStep(max - min, 4);
        max = Mathf.Ceil(max / ystep - 0.001f) * ystep;
        min = Mathf.Floor(min / ystep + 0.001f) * ystep;
        int nY = Mathf.RoundToInt((max - min) / ystep);

        float tEnd = Mathf.Max(tMax, tList[tList.Count - 1]);
        float xstep = NiceStep(tEnd, 5);
        tEnd = Mathf.Ceil(tEnd / xstep - 0.001f) * xstep;

        float X(float tt) { return p.x + tt / tEnd * p.width; }
        float Y(float val) { return p.yMax - (val - min) / (max - min) * p.height; }

        GL.PushMatrix();
        mat.SetPass(0);
        GL.LoadPixelMatrix();

        GL.Begin(GL.LINES);
        GL.Color(new Color(1, 1, 1, 0.2f));
        for (int i = 0; i <= nY; i++) { float val = Tick(min, ystep, i); Seg(p.x, Y(val), p.xMax, Y(val)); }
        for (float tv = 0; tv <= tEnd + 0.001f; tv += xstep) Seg(X(tv), p.y, X(tv), p.yMax);

        //axe vertical à t = 0
        GL.Color(Color.white);
        Seg(p.x, Y(0f), p.xMax, Y(0f));
        Seg(p.x, p.y, p.x, p.yMax);

        GL.Color(col);
        for (int off = -1; off <= 1; off++)
            for (int i = 1; i < data.Count; i++)
                Seg(X(tList[i - 1]), Y(data[i - 1]) + off, X(tList[i]), Y(data[i]) + off);
        GL.End();
        GL.PopMatrix();

        // état : en chute/au repos / fin de simulation
        string state;
        if (landed)
        {
            state = float.IsNaN(before)
                ? "au repos"
                : "au repos, avant impact : " + before.ToString("F2");
        }
        else if (finished)
        {
            state = "fin de la simulation";
        }
        else
        {
            float diff = data[data.Count - 1] - data[Mathf.Max(0, data.Count - 6)];
            state = diff > 0.001f ? "augmente" : diff < -0.001f ? "diminue" : "stable";
        }

        style.alignment = TextAnchor.MiddleLeft;
        style.normal.textColor = col;
        GUI.Label(new Rect(r.x + 10, r.y + 3, r.width, 24),
                  name + " = " + data[data.Count - 1].ToString("F2") + " " + unit + "   (" + state + ")", style);

        style.normal.textColor = Color.white;
        style.alignment = TextAnchor.MiddleRight;
        string yf = ystep >= 1f ? "F0" : "F1";
        for (int i = 0; i <= nY; i++)
        {
            float val = Tick(min, ystep, i);
            GUI.Label(new Rect(r.x + 2, Y(val) - 12, 54, 24), val.ToString(yf), style);
        }

        style.alignment = TextAnchor.MiddleCenter;
        string xf = xstep >= 1f ? "F0" : "F1";
        for (float tv = 0; tv <= tEnd + 0.001f; tv += xstep)
            GUI.Label(new Rect(X(tv) - 30, p.yMax + 3, 60, 24), tv.ToString(xf) + " s", style);
    }

    void Seg(float x0, float y0, float x1, float y1)
    {
        GL.Vertex3(x0, y0, 0);
        GL.Vertex3(x1, y1, 0);
    }
}