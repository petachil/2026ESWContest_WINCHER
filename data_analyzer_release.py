import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 0. 설정 변수
# ==========================================
EVAL_START_M = 1.3 # 평가 시작 구간 (m)
EVAL_END_M = 1.985    # 평가 종료 구간 (m)

# 동일 조건 임무 완료 기준 설정
TASK_START_LEN = 0.00  # 출발 길이 (0m)
TASK_END_LEN = 0.015    # 복귀 완료 길이 (1.5cm)

# 제외할 파일명 키워드
EXCLUDE_KEYWORDS = []

# ==========================================
# 1. 동일 임무 조건 소요 시간 계산 함수
# ==========================================
def calculate_task_duration(df, start_len=0.00, end_len=0.15):
    """
    0m에서 하강을 시작하여 최저점을 찍고 다시 복귀하여 0.015m에 도달할 때까지의 소요 시간 계산
    """
    # 1. 출발 시점 검색 (Length가 0m 초과하여 증가하기 시작하는 index)
    start_candidates = df[df['Length'] > start_len].index
    if len(start_candidates) == 0:
        return np.nan
    start_idx = start_candidates[0]

    # 2. 최저점(최대 길이) 시점 검색
    max_len_idx = df['Length'].idxmax()

    # 최저점 이전이라면 의미 있는 하강/복귀 프로파일이 아님
    if max_len_idx <= start_idx:
        return np.nan

    # 3. 최저점 이후 상승(복귀) 구간 중에서 목표 길이(0.015m) 이하로 들어오는 첫 시점 검색
    after_peak_df = df.loc[max_len_idx:]
    end_candidates = after_peak_df[after_peak_df['Length'] <= end_len].index

    if len(end_candidates) == 0:
        # 복귀 시 0.015m에 도달하지 못했다면 최저점 이후 최소 길이 지점을 종료 지점으로 대체
        end_idx = after_peak_df['Length'].idxmin()
    else:
        end_idx = end_candidates[0]

    # 출발 index부터 완료 index까지의 dt(ms) 총합 -> sec 변환
    task_df = df.loc[start_idx:end_idx]
    duration_sec = np.sum(task_df['dt']) / 1000.0

    return duration_sec

# ==========================================
# 2. 지표 연산 함수
# ==========================================
def calculate_metrics(df_full, df_range):
    # 1) 동일 조건 임무 소요 시간 (0m 출발 -> 최저점 -> 0.015m 복귀)
    task_duration_sec = calculate_task_duration(df_full, start_len=TASK_START_LEN, end_len=TASK_END_LEN)

    if df_range.empty:
        return task_duration_sec, np.nan, np.nan, np.nan, np.nan

    # dt(ms) -> dt(sec) 변환 (구간용)
    dt_sec_range = df_range['dt'] / 1000.0

    # 2) RMSE (deg) - 평가 구간
    rmse = np.sqrt(np.mean(df_range['ThetaMag_deg'] ** 2))
    
    # 3) Peak (deg) - 평가 구간
    peak = np.max(np.abs(df_range['ThetaMag_deg']))
    
    # 4) Sway Signal Energy (deg² · s) - 평가 구간
    sway_energy = np.sum((df_range['ThetaMag_deg'] ** 2) * dt_sec_range)
    
    # 5) Control Effort Energy (cmd² · s) - 평가 구간
    control_energy = np.sum((df_range['MotorCmd'] ** 2) * dt_sec_range)
    
    return task_duration_sec, rmse, peak, sway_energy, control_energy

# ==========================================
# 3. 데이터 로드 및 누적 거리 계산
# ==========================================
def load_csv_data(fpath):
    df_raw = pd.read_csv(fpath, comment='#', header=None)
    
    if not str(df_raw.iloc[0, 0]).replace('.', '', 1).isdigit():
        df_raw = df_raw.iloc[1:].reset_index(drop=True)

    df = pd.DataFrame()
    # 열 매핑: A=0(Time), B=1(dt), K=10(ThetaMag), L=11(Length), Q=16(MotorCmd), T=19(SMCState)
    df['Time']     = df_raw[0].astype(float)
    df['dt']       = df_raw[1].astype(float)
    df['ThetaMag'] = df_raw[10].astype(float)
    df['Length']   = df_raw[11].astype(float)
    df['MotorCmd'] = df_raw[16].astype(float)
    df['SMCState'] = df_raw[19].astype(float)

    # rad -> deg
    df['ThetaMag_deg'] = df['ThetaMag'] * (180.0 / np.pi)

    # Cumulative Distance (0m -> 1m -> 2m)
    length_diff = np.abs(np.diff(df['Length'], prepend=df['Length'].iloc[0]))
    df['CumDistance'] = df['Length'].iloc[0] + np.cumsum(length_diff)

    return df

# ==========================================
# 4. 데이터 일괄 처리 및 구간 분석
# ==========================================
def process_experiment_data(target_folder, start_m=0.75, end_m=1.25, exclude_keywords=None):
    if exclude_keywords is None:
        exclude_keywords = []

    if not os.path.exists(target_folder):
        print(f"❌ 폴더를 찾을 수 없습니다: {target_folder}")
        return

    save_dir = os.path.join(target_folder, f"plots_{start_m}_to_{end_m}m")
    os.makedirs(save_dir, exist_ok=True)

    csv_files = []
    for root, dirs, files in os.walk(target_folder):
        for file in files:
            if file.lower().endswith(".csv"):
                csv_files.append(os.path.join(root, file))
    
    groups = {'smc_off': [], 'smc_on': [], 'tinyml_on': []}
    excluded_count = 0

    for fpath in csv_files:
        fname = os.path.basename(fpath)
        fname_lower = fname.lower()

        if any(kw.lower() in fname_lower for kw in exclude_keywords):
            print(f"🚫 [제외됨] 키워드 매칭 파일 제외: {fname}")
            excluded_count += 1
            continue

        if 'tinyml' in fname_lower or 'ml' in fname_lower:
            groups['tinyml_on'].append(fpath)
        elif 'smcoff' in fname_lower or 'off' in fname_lower:
            groups['smc_off'].append(fpath)
        elif 'smcon' in fname_lower or 'smc' in fname_lower:
            groups['smc_on'].append(fpath)

    print(f"\n📂 감지 및 그룹화 결과 (총 제외된 파일: {excluded_count}개):")
    print(f"   - SMC OFF  : {len(groups['smc_off'])}개")
    print(f"   - SMC ON   : {len(groups['smc_on'])}개")
    print(f"   - TinyML ON: {len(groups['tinyml_on'])}개")
    print(f"🎯 설정된 평가 구간: {start_m}m ~ {end_m}m\n")

    summary_results = []
    plt.ion()

    for group_name, file_list in groups.items():
        for fpath in file_list:
            fname = os.path.basename(fpath)
            
            try:
                df = load_csv_data(fpath)
            except Exception as e:
                print(f"⚠️ {fname} 읽기 실패: {e}")
                continue

            # 평가 구간 슬라이싱
            df_range = df[(df['CumDistance'] >= start_m) & (df['CumDistance'] <= end_m)].copy().reset_index(drop=True)

            if df_range.empty:
                print(f"⚠️ {fname}: {start_m}m~{end_m}m 구간 데이터가 없습니다.")
                continue

            # 지표 계산 (동일 조건 임무 소요시간 + 평가구간 성능)
            task_duration, rmse_range, peak_range, sway_energy, ctrl_energy = calculate_metrics(df, df_range)
            
            summary_results.append({
                'Group': group_name,
                'FileName': fname,
                'Task_Duration_0to0.015m (s)': task_duration,
                f'RMSE_{start_m}_{end_m} (deg)': rmse_range,
                f'Peak_{start_m}_{end_m} (deg)': peak_range,
                f'Sway_Energy_{start_m}_{end_m} (deg²·s)': sway_energy,
                f'Ctrl_Energy_{start_m}_{end_m} (cmd²·s)': ctrl_energy,
                'Data_Points': len(df_range)
            })

            # 시각화
            fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
            fig.suptitle(f"Run: {fname} [{group_name.upper()}] (Eval: {start_m}m ~ {end_m}m)", fontsize=12, fontweight='bold')

            axs[0].plot(df['CumDistance'], df['ThetaMag_deg'], label='Theta Mag (deg)', color='red', linewidth=0.5)
            axs[0].axhline(y=4.0, color='gray', linestyle=':', label='Threshold (4°)')
            axs[0].set_ylabel('Theta Mag [deg]')
            axs[0].grid(True, linestyle='--', alpha=0.6)
            axs[0].legend(loc='upper right')

            smc_binary = (df['SMCState'] == 1).astype(int)
            axs[1].step(df['CumDistance'], smc_binary, where='post', label='SMC Active (1/0)', color='green', linewidth=0.5)
            axs[1].set_xlabel('Cumulative Distance [m]')
            axs[1].set_ylabel('SMC State')
            axs[1].set_ylim([-0.2, 1.2])
            axs[1].set_yticks([0, 1])
            axs[1].grid(True, linestyle='--', alpha=0.6)
            axs[1].legend(loc='upper right')

            axs[0].axvspan(start_m, end_m, color='cyan', alpha=0.2, label=f'Eval Region ({start_m}m~{end_m}m)')
            axs[1].axvspan(start_m, end_m, color='cyan', alpha=0.2)

            plt.tight_layout()
            
            save_path = os.path.join(save_dir, f"{os.path.splitext(fname)[0]}_{start_m}_{end_m}m.png")
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            plt.draw()
            plt.pause(0.1)
            plt.close(fig)

    plt.ioff()

    res_df = pd.DataFrame(summary_results)
    if not res_df.empty:
        print("=========================================================================================")
        print(f"📊 [Trimmed 적용 - 동일 조건(0m->최저점->0.015m) 임무 소요 시간 및 구간 결과]")
        print("=========================================================================================")
        print(res_df.to_string(index=False))

        print("\n=========================================================================================")
        print(f"📈 [Trimmed 적용 - 제어 방식별 평균 비교 (동일 기준 소요시간 포함)]")
        print("=========================================================================================")
        col_task_time = 'Task_Duration_0to0.015m (s)'
        col_rmse = f'RMSE_{start_m}_{end_m} (deg)'
        col_peak = f'Peak_{start_m}_{end_m} (deg)'
        col_sway = f'Sway_Energy_{start_m}_{end_m} (deg²·s)'
        col_ctrl = f'Ctrl_Energy_{start_m}_{end_m} (cmd²·s)'
        
        # 소요 시간 포함 평균 비교
        print(res_df.groupby('Group')[[col_task_time, col_rmse, col_peak, col_sway, col_ctrl]].mean())

if __name__ == "__main__":
    TARGET_PATH = r"C:\Users" #<<저장 주소 입력
    
    process_experiment_data(
        TARGET_PATH, 
        start_m=EVAL_START_M, 
        end_m=EVAL_END_M, 
        exclude_keywords=EXCLUDE_KEYWORDS
    )