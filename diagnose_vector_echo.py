"""Synthetic fixed-pose wall-echo feasibility check. No robot or microphone access."""
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import correlate, correlation_lags

FS = 15625
C = 343.0
MICS = np.array([[-.014,-.011,0],[-.014,.011,0],[.014,-.011,0],[.014,.011,0]])
SOURCE = np.array([.010, 0, .012])
DIRECT = np.linalg.norm(MICS-SOURCE, axis=1)
TIME = np.arange(420)/FS


def chirp(t):
    duration = .004
    window = np.sin(np.pi*np.clip(t/duration, 0, 1))**2
    return window*np.sin(2*np.pi*(2000*t + .5*4000/duration*t*t))*(t >= 0)*(t < duration)


def capture(rng, wall, bearing=0):
    """Analytic audible waveform sampled on the raw mic clock. Vary playback start and coupling."""
    start = rng.uniform(.0015,.003)
    n = np.array([np.cos(bearing), np.sin(bearing), 0])
    image = SOURCE + 2*(wall or 0)*n
    tracks = []
    for mic, dist in zip(MICS,DIRECT):
        t = TIME-start-dist/C
        y = chirp(t) + rng.uniform(.12,.18)*chirp(t-.0008)
        if wall is not None:
            y += .035*chirp(TIME-start-np.linalg.norm(image-mic)/C)
        tracks.append(y+rng.normal(0,.011,len(t)))
    return tracks


def peak(power, indices):
    k = indices[np.argmax(power[indices])]
    left, mid, right = power[k-1:k+2]
    offset = np.clip(.5*(left-right)/(left-2*mid+right),-.5,.5) if mid > left and mid > right else 0
    return k+offset, mid


def measure(tracks):
    # Anchor on the direct arrival in each channel. Ignore paths within
    # 1.4 ms of it so that the simulated enclosure reflection is excluded.
    ref = chirp(np.arange(64)/FS)
    paths, strengths = [], []
    for track, direct in zip(tracks, DIRECT):
        matched = correlate(track,ref,mode="full") / (ref@ref)
        lags = correlation_lags(len(track),len(ref))
        k, _ = peak(matched, np.flatnonzero((lags>0)&(lags<80)))
        j, strength = peak(matched, np.flatnonzero((np.arange(len(lags))>k+.0014*FS)
                                                 &(np.arange(len(lags))<k+.008*FS)))
        paths.append(direct+C*(j-k)/FS)
        strengths.append(strength)
    return np.array(paths), min(strengths)


def wall_fit(paths):
    def residual(p):
        image = SOURCE + 2*p[0]*np.array([np.cos(p[1]),np.sin(p[1]),0])
        return np.linalg.norm(image-MICS,axis=1)-paths
    # Multiple seeds avoid an artificial preference for the front hemisphere.
    sols = [least_squares(residual,[np.mean(paths)/2,a]) for a in (0, np.pi/2, np.pi, -np.pi/2)]
    r, angle = min(sols,key=lambda s:s.cost).x
    return abs(r), (angle+(np.pi if r<0 else 0))


def main():
    rng=np.random.default_rng(119)
    threshold=max(measure(capture(rng,None))[1] for _ in range(100))
    false=sum(measure(capture(rng,None))[1]>threshold for _ in range(100))
    print(f"Independent no-wall detections: {false}/100. Strength threshold {threshold:.4f}")
    for wall in (.20,.40,.80):
        errors=[]
        for bearing in np.radians([-45,0,45]):
            for _ in range(20):
                paths,strength=measure(capture(rng,wall,bearing))
                if strength<=threshold:
                    continue
                r,a=wall_fit(paths)
                errors.append([1000*abs(r-wall),abs(np.degrees(np.arctan2(np.sin(a-bearing),np.cos(a-bearing))))])
        print(wall, len(errors), "/ 60 detected; median absolute range (mm), bearing (deg):",
              np.median(errors,axis=0).round(2) if errors else "n/a")


if __name__ == "__main__":
    main()
