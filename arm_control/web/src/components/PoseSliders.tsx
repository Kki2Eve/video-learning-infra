import type { PoseControlsState } from "../hooks/usePoseControls";

interface SliderDefinition {
  key: keyof PoseControlsState;
  label: string;
  min: number;
  max: number;
  step: number;
}

const SLIDERS: SliderDefinition[] = [
  { key: "tx", label: "tx", min: -1.0, max: 1.0, step: 0.01 },
  { key: "ty", label: "ty", min: -1.0, max: 1.0, step: 0.01 },
  { key: "tz", label: "tz", min: -1.0, max: 1.5, step: 0.01 },
  { key: "roll", label: "roll", min: -3.14, max: 3.14, step: 0.01 },
  { key: "pitch", label: "pitch", min: -3.14, max: 3.14, step: 0.01 },
  { key: "yaw", label: "yaw", min: -3.14, max: 3.14, step: 0.01 },
];

interface PoseSlidersProps {
  controls: PoseControlsState;
  onChange: (key: keyof PoseControlsState, value: number) => void;
}

export function PoseSliders({ controls, onChange }: PoseSlidersProps) {
  return (
    <div className="slider-list">
      {SLIDERS.map((slider) => (
        <label key={slider.key} className="slider-row">
          <div className="slider-header">
            <span>{slider.label}</span>
            <span>{controls[slider.key].toFixed(2)}</span>
          </div>
          <input
            type="range"
            min={slider.min}
            max={slider.max}
            step={slider.step}
            value={controls[slider.key]}
            onChange={(event) => onChange(slider.key, Number(event.target.value))}
          />
        </label>
      ))}
    </div>
  );
}
